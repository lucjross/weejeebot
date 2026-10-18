package com.lucjross.ouija.client

import com.lucjross.ouija.DeepgramConfig
import io.ktor.client.*
import io.ktor.client.plugins.websocket.*
import io.ktor.client.request.*
import io.ktor.http.*
import io.ktor.websocket.*
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.Flow
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import io.github.oshai.kotlinlogging.KotlinLogging

@Serializable
data class DeepgramMessage(
    val type: String? = null,
    @SerialName("is_final") val isFinal: Boolean = false,
    @SerialName("speech_final") val speechFinal: Boolean = false,
    val channel: DeepgramChannel? = null
) {
    val transcript: String get() = channel?.alternatives?.firstOrNull()?.transcript.orEmpty()
}

@Serializable
data class DeepgramChannel(val alternatives: List<DeepgramAlternative> = emptyList())

@Serializable
data class DeepgramAlternative(val transcript: String = "")

/**
 * Joins Deepgram's final results into whole utterances. Interim results are not requested, so every result with
 * `is_final` is settled text; a long utterance can arrive as several of them, and `speech_final` marks the pause that
 * ends it.
 */
class UtteranceAssembler {
    private val parts = mutableListOf<String>()

    /** Returns the finished utterance when [message] ends one, otherwise null. */
    fun accept(message: DeepgramMessage): String? {
        if (message.type == "UtteranceEnd") return flush()
        if (message.type != null && message.type != "Results") return null
        if (message.isFinal && message.transcript.isNotBlank()) parts += message.transcript.trim()
        return if (message.speechFinal) flush() else null
    }

    private fun flush(): String? {
        if (parts.isEmpty()) return null
        return parts.joinToString(" ").also { parts.clear() }
    }
}

/** Streams microphone audio to Deepgram over one WebSocket session and reports each finished utterance. */
class DeepgramClient(private val http: HttpClient, private val config: DeepgramConfig) {

    private val logger = KotlinLogging.logger {}
    private val json = Json { ignoreUnknownKeys = true }

    companion object {
        const val ENDPOINT = "wss://api.deepgram.com/v1/listen"
        const val KEEP_ALIVE_MS = 5_000L
        private const val KEEP_ALIVE = """{"type":"KeepAlive"}"""
        private const val CLOSE_STREAM = """{"type":"CloseStream"}"""
    }

    fun url(): String = URLBuilder(ENDPOINT).apply {
        parameters.append("model", config.model)
        parameters.append("language", config.language)
        parameters.append("encoding", "linear16")
        parameters.append("sample_rate", "16000")
        parameters.append("channels", "1")
        parameters.append("smart_format", "true")
        parameters.append("punctuate", "true")
        parameters.append("interim_results", "false")
        parameters.append("endpointing", config.endpointingMs.toString())
        config.keyterms.forEach { parameters.append("keyterm", it) }
    }.buildString()

    /** Returns when the session ends: the audio ran out, Deepgram closed the socket, or the caller cancelled. */
    suspend fun stream(audio: Flow<ByteArray>, onUtterance: suspend (String) -> Unit) {
        http.webSocket(url(), request = { header(HttpHeaders.Authorization, "Token ${config.apiKey}") }) {
            logger.info { "connected to Deepgram" }
            val assembler = UtteranceAssembler()

            val sender = launch {
                audio.collect { send(Frame.Binary(true, it)) }
                send(Frame.Text(CLOSE_STREAM))
            }
            val keepAlive = launch {
                while (isActive) {
                    delay(KEEP_ALIVE_MS)
                    send(Frame.Text(KEEP_ALIVE))
                }
            }

            try {
                for (frame in incoming) {
                    if (frame !is Frame.Text) continue
                    val message = try {
                        json.decodeFromString<DeepgramMessage>(frame.readText())
                    } catch (e: Exception) {
                        logger.warn(e) { "unreadable Deepgram message" }
                        continue
                    }
                    assembler.accept(message)?.let {
                        logger.debug { "[transcript] $it" }
                        onUtterance(it)
                    }
                }
            } finally {
                sender.cancel()
                keepAlive.cancel()
            }
        }
    }
}
