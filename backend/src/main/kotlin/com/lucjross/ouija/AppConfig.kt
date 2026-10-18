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

data class AppConfig(val deepgram: DeepgramConfig, val bot: BotConfig) {
    companion object {
        const val DEFAULT_NANO_URL = "http://192.168.68.106"

        fun from(config: ApplicationConfig) = AppConfig(
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
            )
        )
    }
}
