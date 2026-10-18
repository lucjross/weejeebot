package com.lucjross.ouija

import com.lucjross.ouija.client.BotClient
import com.lucjross.ouija.client.DeepgramClient
import com.lucjross.ouija.daemon.OuijaState
import com.lucjross.ouija.daemon.OuijaTimings
import com.lucjross.ouija.daemon.TranscriptionDaemon
import io.github.oshai.kotlinlogging.KotlinLogging
import io.ktor.client.*
import io.ktor.client.engine.cio.*
import io.ktor.client.plugins.websocket.*
import io.ktor.http.*
import io.ktor.server.application.*
import io.ktor.server.plugins.calllogging.*
import io.ktor.server.response.*
import io.ktor.server.routing.*
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel

private val logger = KotlinLogging.logger {}

fun Application.module() {
    install(CallLogging)

    val config = AppConfig.from(environment.config)
    val http = HttpClient(CIO) { install(WebSockets) }
    val scope = CoroutineScope(SupervisorJob())

    val bot = BotClient(http, config.bot.baseUrl)
    val ouijaState = OuijaState(
        bot, scope,
        timings = OuijaTimings(msPerLetter = config.bot.secondsPerLetter * 1_000L)
    )

    if (config.deepgram.apiKey.isBlank()) {
        logger.warn { "DEEPGRAM_API_KEY is not set; the microphone is not being transcribed" }
    } else {
        TranscriptionDaemon(DeepgramClient(http, config.deepgram), Mic(), ouijaState).start(scope)
    }

    monitor.subscribe(ApplicationStopping) {
        scope.cancel()
        http.close()
    }

    routing {
        get("/health") {
            call.respondText("ok")
        }

        // for the Nano to call when the bot has finished spelling, so the next question is taken without waiting
        post("/bot/idle") {
            ouijaState.botIdle()
            call.respond(HttpStatusCode.NoContent)
        }
    }
}
