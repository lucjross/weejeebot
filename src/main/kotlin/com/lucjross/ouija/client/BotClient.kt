package com.lucjross.ouija.client

import mu.KotlinLogging
import org.springframework.http.HttpMethod
import org.springframework.stereotype.Component
import org.springframework.web.reactive.function.BodyInserters
import org.springframework.web.reactive.function.client.WebClient
import reactor.core.publisher.Mono

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