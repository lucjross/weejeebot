package com.lucjross.ouija.client

import com.lucjross.ouija.LlmConfig
import io.github.oshai.kotlinlogging.KotlinLogging
import io.ktor.client.*
import io.ktor.client.request.*
import io.ktor.client.statement.*
import io.ktor.http.*
import kotlinx.serialization.json.*

/** Asks Claude for the cryptic one-word answer. */
class LlmClient(private val http: HttpClient, private val config: LlmConfig) {

    private val logger = KotlinLogging.logger {}

    // answers already given to each question, so asking again gets a different one
    private val previousAnswers = mutableMapOf<String, MutableList<String>>()

    companion object {
        const val SYSTEM_PROMPT = "You are a psychic speaking through a Ouija board. " +
                "Answer the question with a single cryptic word of fewer than ten letters. " +
                "The word must actually answer the question: a place for where, a name for who, a time for when, " +
                "a reason for why, a number for how many, yes or no for a yes/no question, and something " +
                "specific and surprising for anything else. Do not fall back on spooky filler like whisper, " +
                "shadow, spirit or darkness. " +
                "Never refuse, hedge or explain: if the question cannot be known, give your best guess as a single " +
                "word. Reply with only that word: no punctuation, no other text."
        // answers of the last few questions are kept out of the next ones, so the same word cannot keep coming back
        private const val RECENT_ANSWERS = 8
        private val ONE_WORD = Regex("[a-z0-9]{2,12}")
    }

    private val recentAnswers = ArrayDeque<String>()

    /** The answer as one lowercase word, or null if the call failed (the caller then falls back). */
    suspend fun ask(question: String): String? {
        val previous = previousAnswers.getOrPut(question) { mutableListOf() }
        val avoid = (previous + recentAnswers).distinct()
        val userText = if (avoid.isEmpty()) question
        else "$question (don't say ${avoid.joinToString(" or ")})"
        try {
            val response = http.post(config.baseUrl + "/v1/messages") {
                header("x-api-key", config.apiKey)
                header("anthropic-version", "2023-06-01")
                contentType(ContentType.Application.Json)
                setBody(buildJsonObject {
                    put("model", config.model)
                    put("max_tokens", 16)
                    put("temperature", 1.0)
                    // thinking is on by default and would use up max_tokens before any answer text
                    putJsonObject("thinking") { put("type", "disabled") }
                    put("system", SYSTEM_PROMPT)
                    putJsonArray("messages") {
                        addJsonObject {
                            put("role", "user")
                            put("content", userText)
                        }
                    }
                }.toString())
            }
            val body = response.bodyAsText()
            if (!response.status.isSuccess()) {
                logger.error { "Messages API: ${response.status}: $body" }
                return null
            }
            val text = Json.parseToJsonElement(body).jsonObject["content"]?.jsonArray
                ?.firstNotNullOfOrNull { it.jsonObject["text"]?.jsonPrimitive?.contentOrNull }
            // the whole reply has to be one word; a refusal or a sentence is not an answer
            val word = text?.trim()?.trim('.', '!', '"', '\'')?.lowercase()?.takeIf { ONE_WORD.matches(it) }
            if (word == null) {
                logger.warn { "Messages API reply is not a single word: \"$text\"" }
            }
            return word
                ?.also {
                    previous.add(it)
                    recentAnswers.addLast(it)
                    if (recentAnswers.size > RECENT_ANSWERS) recentAnswers.removeFirst()
                }
        } catch (e: Exception) {
            logger.error(e) { "Messages API call failed: ${e.message}" }
            return null
        }
    }
}
