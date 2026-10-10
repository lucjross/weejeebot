package com.lucjross.ouija

import io.ktor.server.application.*
import io.ktor.server.plugins.calllogging.*
import io.ktor.server.response.*
import io.ktor.server.routing.*

fun Application.module() {
    install(CallLogging)

    routing {
        get("/health") {
            call.respondText("ok")
        }
    }
}
