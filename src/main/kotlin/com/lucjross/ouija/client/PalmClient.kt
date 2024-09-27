package com.lucjross.ouija.client

import com.google.ai.generativelanguage.v1beta3.GenerateTextRequest
import com.google.ai.generativelanguage.v1beta3.TextPrompt
import com.google.ai.generativelanguage.v1beta3.TextServiceClient
import com.google.ai.generativelanguage.v1beta3.TextServiceSettings
import com.google.api.gax.core.FixedCredentialsProvider
import com.google.api.gax.grpc.InstantiatingGrpcChannelProvider
import com.google.api.gax.rpc.FixedHeaderProvider
import com.lucjross.ouija.AppProperties
import mu.KotlinLogging
import org.springframework.stereotype.Component
import reactor.core.publisher.Mono
import reactor.core.scheduler.Schedulers
import java.util.concurrent.ConcurrentHashMap

@Component
class PalmClient(private val textServiceClient: TextServiceClient) {

    private val logger = KotlinLogging.logger {}

    companion object {

        const val PROMPT_FORMAT = "Pretending you are a psychic, " +
                "provide a cryptic answer to the following question " +
                "using a single word with fewer than ten characters: %s"
        const val DONT_SAY_FORMAT = " (don't say %s)"
        const val OR_PHRASE = " or "
    }

    private val previousQandAs = mutableMapOf<String, MutableList<String>>()

    fun ask(q: String): String? {
        val previousAnswers = previousQandAs.computeIfAbsent(q) { mutableListOf() }
        val prompt = TextPrompt.newBuilder()
            .setText(String.format(PROMPT_FORMAT, q)
                .let {
                    if (previousAnswers.isEmpty())
                        it
                    else
                        it + String.format(DONT_SAY_FORMAT, previousAnswers.joinToString(OR_PHRASE))
                })
            .build()
        val req = GenerateTextRequest.newBuilder()
            .setModel("models/text-bison-001")
            .setPrompt(prompt)
            .setTemperature(1.0f)
            .setCandidateCount(1)
            .build()

        return Mono.fromSupplier {
            val res = textServiceClient.generateText(req)
            logger.info("generateText response: {}", res)
            val returnedText = res.candidatesList.firstOrNull()
            if (returnedText == null) {
                logger.warn("no text")
                null
            } else returnedText.output
                .let { "[A-Za-z]+".toRegex().find(it) }
                ?.value
                ?.lowercase()
                .also { it?.let(previousAnswers::add) }
        }
            .subscribeOn(Schedulers.boundedElastic())
            .doOnError { logger.error("generateText error", it) }
            .block()
    }
}