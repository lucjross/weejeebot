package com.lucjross.ouija

import io.awspring.cloud.autoconfigure.context.properties.AwsCredentialsProperties
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import mu.KotlinLogging
import org.reactivestreams.Publisher
import org.reactivestreams.Subscriber
import org.reactivestreams.Subscription
import org.springframework.beans.factory.config.ConfigurableBeanFactory
import org.springframework.context.ApplicationContext
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration
import org.springframework.context.annotation.Scope
import org.springframework.http.HttpMethod
import org.springframework.stereotype.Component
import org.springframework.web.reactive.function.BodyInserters
import org.springframework.web.reactive.function.client.WebClient
import reactor.core.publisher.Mono
import software.amazon.awssdk.auth.credentials.DefaultCredentialsProvider
import software.amazon.awssdk.core.SdkBytes
import software.amazon.awssdk.regions.Region
import software.amazon.awssdk.services.transcribestreaming.TranscribeStreamingAsyncClient
import software.amazon.awssdk.services.transcribestreaming.model.*
import java.io.InputStream
import java.nio.ByteBuffer
import java.time.Clock
import java.time.Instant
import java.util.concurrent.*
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import javax.annotation.PostConstruct
import javax.sound.sampled.*

@Configuration
class QAndAConfiguration {

    companion object {
        private val mic = getMic()

        init {
            mic.start()
        }
    }

    @Bean
    fun transcribeStreamingAsyncClient(awsCredentialsProperties: AwsCredentialsProperties): TranscribeStreamingAsyncClient =
        TranscribeStreamingAsyncClient.builder()
            .credentialsProvider(
                DefaultCredentialsProvider.builder()
                    .profileName(awsCredentialsProperties.profileName)
                    .build()
            )
            .region(Region.US_WEST_2)
            .build()

    @Bean
    fun nanoClient(): WebClient {
        return WebClient.create("http://192.168.4.101");
    }

    @Bean
    fun scheduledExecutorService(): ScheduledExecutorService {
        return Executors.newSingleThreadScheduledExecutor()
    }

    @Bean
    fun audioInputStream(): AudioInputStream {
        return AudioInputStream(mic)
    }

    @Bean(destroyMethod = "destroy")
    fun audioStreamPublisher(audioInputStream: AudioInputStream): AudioStreamPublisher {
        return AudioStreamPublisher(audioInputStream)
    }
}

fun getMic(): TargetDataLine {
    val format = AudioFormat(16_000f, 16, 1, true, false)
    val dataLineInfo = DataLine.Info(TargetDataLine::class.java, format)
    if (!AudioSystem.isLineSupported(dataLineInfo)) {
        throw IllegalStateException()
    }

    val dataLine = AudioSystem.getLine(dataLineInfo) as TargetDataLine
    dataLine.open(format, dataLine.bufferSize)
    return dataLine
}


class AudioStreamPublisher(private val inputStream: InputStream) :
        Publisher<AudioStream> {

    companion object {
        const val CHUNK_SIZE_BYTES = 1024
    }

    private lateinit var audioStreamSubscription: AudioStreamSubscription

    override fun subscribe(subscriber: Subscriber<in AudioStream>) {
        audioStreamSubscription = AudioStreamSubscription(subscriber, inputStream)
        subscriber.onSubscribe(audioStreamSubscription)
    }

    fun destroy() {
        audioStreamSubscription.cancel()
    }


    class AudioStreamSubscription(
        private val subscriber: Subscriber<in AudioStream>,
        private val inputStream: InputStream
    ) : Subscription {

        private val logger = KotlinLogging.logger {}

        private val executorService: ExecutorService = Executors.newFixedThreadPool(1)
        private val demand = AtomicLong(0)
        private val cancelled = AtomicBoolean(false)

        override fun request(n: Long) {
            if (n <= 0) {
                subscriber.onError(IllegalArgumentException("n <= 0"))
            }

            demand.getAndAdd(n)

            executorService.submit {
                if (cancelled.get()) {
                    return@submit
                }

                try {
                    do {
                        val audioBuffer = getNextEvent()
                        if (audioBuffer.remaining() > 0) {
                            val audioEvent = AudioEvent.builder()
                                .audioChunk(SdkBytes.fromByteBuffer(audioBuffer)).build()
                            subscriber.onNext(audioEvent)
                        } else {
                            subscriber.onComplete()
                            break
                        }
                    } while (!cancelled.get() && demand.decrementAndGet() > 0)
                } catch (e: TranscribeStreamingException) {
                    logger.error(e.message, e)
                    subscriber.onError(e)
                }
            }
        }

        override fun cancel() {
            logger.debug { "cancel" }
            cancelled.set(true)
        }

        private fun getNextEvent(): ByteBuffer {
            val audioBytes = ByteArray(CHUNK_SIZE_BYTES)
            val len = inputStream.read(audioBytes)
            val audioBuffer = if (len <= 0) {
                ByteBuffer.allocate(0)
            } else {
                ByteBuffer.wrap(audioBytes, 0, len)
            }

            return audioBuffer
        }
    }
}


@Component
class TranscriptionDaemon(
    private val ouijaState: OuijaState,
    private val transcribeStreamingAsyncClient: TranscribeStreamingAsyncClient,
    private val applicationContext: ApplicationContext
) {

    private val logger = KotlinLogging.logger {}

    private val t = Thread {
        val request = StartStreamTranscriptionRequest.builder()
            .mediaEncoding(MediaEncoding.PCM)
            .languageCode(LanguageCode.EN_US)
            .mediaSampleRateHertz(16_000)
            .vocabularyName("Ouija01")
            .build()

        val handler = StartStreamTranscriptionResponseHandler.builder()
            .subscriber { transcriptEvent ->
                processTranscript((transcriptEvent as TranscriptEvent).transcript())
            }
            .build()

        val publisher = applicationContext.getBean(AudioStreamPublisher::class.java)
        try {
            logger.debug { "starting stream transcription" }
            val transcriptionFuture =
                transcribeStreamingAsyncClient.startStreamTranscription(request, publisher, handler)
            transcriptionFuture.get()
        } catch (e: ExecutionException) {
            logger.warn(e) { e.message }
        } catch (e: TranscribeStreamingException) {
            logger.error(e) { e.message }
        }

        logger.info { "stream transcription done" }
    }

    @PostConstruct
    fun initialize() {
        if (!t.isAlive) {
            t.start()
        }
    }

    fun processTranscript(transcript: Transcript) {
        transcript.results().forEach {
            it.alternatives().forEach { alt ->
                logger.debug { "[transcript] ${alt.transcript()}" }
                ouijaState.heard(alt.transcript());
            }
        }
    }
}

@Component
class OuijaState(
    private val botClient: BotClient,
    private val scheduledExecutorService: ScheduledExecutorService) {

    private val logger = KotlinLogging.logger {}

    private var heardHeyOuija = Instant.EPOCH
    private var answerSent = Instant.EPOCH
    private val clock = Clock.systemDefaultZone()
    private val debouncedAnswers: ConcurrentNavigableMap<String, ScheduledFuture<*>> =
        ConcurrentSkipListMap { o1, o2 -> o1.length - o2.length }

    @Synchronized
    fun heard(transcribed: String) {
        val heyOuijaMatch = Regex("Hey,? Ouija[.!]\\s*").find(transcribed)
        if (heyOuijaMatch != null) {
            if (heardHeyOuija.isBefore(clock.instant().minusSeconds(10))) {
                botClient.sendHeyOuija();

                // this can cancel any running answer
                answerSent = Instant.EPOCH
            }

            heardHeyOuija = clock.instant();
        }

        if (answerSent.isBefore(clock.instant().minusSeconds(5))) {

            var strWithoutPrompt = heyOuijaMatch?.let { transcribed.removePrefix(it.value) } ?: transcribed;

            if (strWithoutPrompt.endsWith("?") && heardHeyOuija.isAfter(clock.instant().minusSeconds(15))) {

                val previousPuncIdx = strWithoutPrompt.removeSuffix("?").lastIndexOfAny(charArrayOf('.', '!', '?')) + 1
                if (previousPuncIdx > 0) {
                    strWithoutPrompt = strWithoutPrompt.substring(previousPuncIdx).trim()
                }

                // debounce by removing previous entries (shorter or same questions) from the scheduled tasks
                var removed: String?
                do {
                    removed = debouncedAnswers.floorKey(strWithoutPrompt)?.apply { debouncedAnswers.remove(this) }
                    removed?.apply { logger.debug("debounced answer: $this") }
                } while (removed != null)

                val future = scheduledExecutorService.schedule({
                    if (debouncedAnswers.containsKey(strWithoutPrompt)) {
                        // occasionally multiple complete questions end up in the queue,
                        // like when it gives two alternatives for an ambiguous word,
                        // so empty the queue upon sending an answer
                        debouncedAnswers.clear()

                        val answerTypes = QuestionTypeNode.match(strWithoutPrompt)
                        val answer = answerTypes.random().answer.invoke()
                        if (answer == null) {
                            botClient.sendTryAgain()
                        } else {
                            logger.debug { "For question \"$strWithoutPrompt\" sending answer \"$answer\"" }
                            botClient.sendAnswer(answer)
                        }

                        answerSent = clock.instant()
                    }
                }, 1, TimeUnit.SECONDS)
                debouncedAnswers[strWithoutPrompt] = future
            }
        }
    }
}

@Component
class BotClient(private val nanoClient: WebClient) {

    private val logger = KotlinLogging.logger {}

    fun sendHeyOuija() {
        nanoClient.method(HttpMethod.POST)
            .uri("/heyOuija")
            .exchangeToMono {
                logger.debug("/heyOuija: ${it.statusCode()}")
                Mono.empty<Unit>()
            }
            .subscribe()
    }

    fun sendAnswer(answer: String) {
        nanoClient.method(HttpMethod.POST)
            .uri("/answer")
            .body(BodyInserters.fromFormData("answer", answer))
            .exchangeToMono {
                logger.debug("/answer: ${it.statusCode()}")
                Mono.empty<Unit>()
            }
            .subscribe()
    }

    fun sendTryAgain() {
        nanoClient.method(HttpMethod.POST)
            .uri("/tryAgain")
            .exchangeToMono {
                logger.debug("/tryAgain: ${it.statusCode()}")
                Mono.empty<Unit>()
            }
            .subscribe()
    }
}
