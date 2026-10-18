package com.lucjross.ouija

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.flowOn
import kotlinx.coroutines.isActive
import javax.sound.sampled.AudioFormat
import javax.sound.sampled.AudioSystem
import javax.sound.sampled.DataLine
import javax.sound.sampled.TargetDataLine

/** The laptop's microphone as a stream of raw PCM chunks: 16 kHz, 16-bit, mono, little-endian. */
class Mic {

    companion object {
        const val SAMPLE_RATE = 16_000
        // 100 ms of audio, inside the 20-250 ms that streaming recognizers like
        const val CHUNK_SIZE_BYTES = SAMPLE_RATE / 10 * 2
    }

    /** Opens the mic when collected and closes it when the collector is cancelled. */
    fun chunks(): Flow<ByteArray> = flow {
        val format = AudioFormat(SAMPLE_RATE.toFloat(), 16, 1, true, false)
        val info = DataLine.Info(TargetDataLine::class.java, format)
        check(AudioSystem.isLineSupported(info)) { "no microphone supports 16 kHz 16-bit mono" }

        val line = AudioSystem.getLine(info) as TargetDataLine
        line.open(format)
        line.start()
        try {
            val buffer = ByteArray(CHUNK_SIZE_BYTES)
            while (currentCoroutineContext().isActive) {
                val n = line.read(buffer, 0, buffer.size)
                if (n > 0) emit(buffer.copyOf(n))
            }
        } finally {
            line.stop()
            line.close()
        }
    }.flowOn(Dispatchers.IO)
}
