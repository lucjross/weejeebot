package com.lucjross.ouija.daemon

import com.lucjross.ouija.AudioStreamPublisher
import mu.KotlinLogging
import org.springframework.context.ApplicationContext
import org.springframework.stereotype.Component
import software.amazon.awssdk.services.transcribestreaming.TranscribeStreamingAsyncClient
import software.amazon.awssdk.services.transcribestreaming.model.*
import java.util.concurrent.ExecutionException
import javax.annotation.PostConstruct

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