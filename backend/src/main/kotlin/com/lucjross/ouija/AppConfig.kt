package com.lucjross.ouija

import io.ktor.server.config.*

data class DeepgramConfig(
    val apiKey: String,
    val model: String,
    val language: String,
    val endpointingMs: Int,
    val keyterms: List<String>
)

data class BotConfig(val baseUrl: String, val secondsPerLetter: Int)

data class LlmConfig(val apiKey: String, val model: String, val baseUrl: String)

/** Where the questions come from: the microphone through Deepgram, or lines typed on stdin. */
enum class InputMode { MIC, STDIN }

data class AppConfig(val input: InputMode, val deepgram: DeepgramConfig, val bot: BotConfig, val llm: LlmConfig) {
    companion object {
        const val DEFAULT_NANO_URL = "http://192.168.68.106"

        fun from(config: ApplicationConfig) = AppConfig(
            input = when (val mode = config.propertyOrNull("ouija.input")?.getString()?.lowercase()) {
                null, "", "mic" -> InputMode.MIC
                "stdin" -> InputMode.STDIN
                else -> error("ouija.input must be mic or stdin, not \"$mode\"")
            },
            deepgram = DeepgramConfig(
                apiKey = config.propertyOrNull("ouija.deepgram.api-key")?.getString().orEmpty(),
                model = config.property("ouija.deepgram.model").getString(),
                language = config.property("ouija.deepgram.language").getString(),
                endpointingMs = config.property("ouija.deepgram.endpointing-ms").getString().toInt(),
                keyterms = config.propertyOrNull("ouija.deepgram.keyterms")?.getList().orEmpty()
            ),
            bot = BotConfig(
                baseUrl = config.propertyOrNull("ouija.bot.base-url")?.getString()?.takeIf { it.isNotBlank() }
                    ?: DEFAULT_NANO_URL,
                secondsPerLetter = config.property("ouija.bot.seconds-per-letter").getString().toInt()
            ),
            llm = LlmConfig(
                apiKey = config.propertyOrNull("ouija.llm.api-key")?.getString().orEmpty(),
                model = config.property("ouija.llm.model").getString(),
                baseUrl = config.property("ouija.llm.base-url").getString()
            )
        )
    }
}
