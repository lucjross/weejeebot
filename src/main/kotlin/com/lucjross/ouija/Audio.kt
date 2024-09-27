package com.lucjross.ouija

import mu.KotlinLogging
import org.reactivestreams.Publisher
import org.reactivestreams.Subscriber
import org.reactivestreams.Subscription
import org.springframework.stereotype.Component
import software.amazon.awssdk.core.SdkBytes
import software.amazon.awssdk.services.transcribestreaming.model.AudioEvent
import software.amazon.awssdk.services.transcribestreaming.model.AudioStream
import software.amazon.awssdk.services.transcribestreaming.model.TranscribeStreamingException
import java.io.InputStream
import java.nio.ByteBuffer
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import javax.sound.sampled.AudioFormat
import javax.sound.sampled.AudioSystem
import javax.sound.sampled.DataLine
import javax.sound.sampled.TargetDataLine

@Component
class Mic {

    companion object {
        private val dataLine = openDataLine()

        init {
            dataLine.start()
        }

        private fun openDataLine(): TargetDataLine {
            val format = AudioFormat(16_000f, 16, 1, true, false)
            val dataLineInfo = DataLine.Info(TargetDataLine::class.java, format)
            if (!AudioSystem.isLineSupported(dataLineInfo)) {
                throw IllegalStateException()
            }

            val dataLine = AudioSystem.getLine(dataLineInfo) as TargetDataLine
            dataLine.open(format, dataLine.bufferSize)
            return dataLine
        }
    }

    fun getMic() = dataLine
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
                                .audioChunk(SdkBytes.fromByteBuffer(audioBuffer))
                                .build()
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