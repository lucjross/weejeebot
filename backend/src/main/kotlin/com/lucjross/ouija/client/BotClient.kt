package com.lucjross.ouija.client

import io.ktor.client.*
import io.ktor.client.request.*
import io.ktor.client.request.forms.*
import io.ktor.client.statement.*
import io.ktor.http.*
import io.github.oshai.kotlinlogging.KotlinLogging

/** What the state machine needs from the bot. Each call returns true when the Nano accepted it. */
interface Bot {
    suspend fun heyOuija(): Boolean
    suspend fun answer(answer: String): Boolean
    suspend fun tryAgain(): Boolean
}

/** Talks to the Nano 33 IoT on the planchette. */
class BotClient(private val http: HttpClient, private val baseUrl: String) : Bot {

    private val logger = KotlinLogging.logger {}

    override suspend fun heyOuija() = send("/heyOuija") { post(baseUrl + "/heyOuija") }

    override suspend fun answer(answer: String) = send("/answer") {
        submitForm(baseUrl + "/answer", Parameters.build { append("answer", answer) })
    }

    override suspend fun tryAgain() = send("/tryAgain") { post(baseUrl + "/tryAgain") }

    private suspend fun send(path: String, call: suspend HttpClient.() -> HttpResponse): Boolean = try {
        val status = http.call().status
        logger.debug { "$path: $status" }
        status.isSuccess()
    } catch (e: Exception) {
        logger.error(e) { "$path failed: ${e.message}" }
        false
    }
}
