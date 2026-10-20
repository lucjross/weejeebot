package com.lucjross.ouija.daemon

import com.lucjross.ouija.QuestionTypeNode
import com.lucjross.ouija.client.Bot
import io.github.oshai.kotlinlogging.KotlinLogging
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock

data class OuijaTimings(
    /** how long after "Hey, Ouija" a question is accepted */
    val wakeWindowMs: Long = 15_000,
    /** wait after a sentence ending in "?" in case the question carries on */
    val settleMs: Long = 1_000,
    /** time the bot is assumed busy per spelled letter, until the Nano can report that it is done */
    val msPerLetter: Long = 6_000,
    /** busy time for a "try again" or a one-letter answer */
    val minBusyMs: Long = 4_000
)

/**
 * Turns utterances from the recognizer into at most one answer at a time.
 *
 * Sleeping -> "Hey, Ouija" wakes it for [OuijaTimings.wakeWindowMs] -> a sentence ending in "?" is the question ->
 * the answer goes to the bot, which is then busy until it has had time to spell it. Everything heard while the bot is
 * busy is ignored, and each answer uses up the wake word, so a follow-up needs a new "Hey, Ouija".
 *
 * [heard] takes final pieces of text (punctuated, settled), not interim results. A question can arrive in several
 * pieces, so the text since the wake word is joined up here.
 */
class OuijaState(
    private val bot: Bot,
    private val scope: CoroutineScope,
    private val now: () -> Long = System::currentTimeMillis,
    private val answerFor: suspend (String) -> String? = { question ->
        QuestionTypeNode.match(question).random().answer.invoke()
    },
    private val timings: OuijaTimings = OuijaTimings()
) {

    private val logger = KotlinLogging.logger {}

    companion object {
        // the wake word as the recognizer tends to write it: "Ouija" is pronounced "wee-jee"
        private val WAKE_WORD = Regex(
            "\\bhey,?\\s+(ouija|oui\\s?ja|ouiji|wee[\\s-]?jee|weegee|wee\\s?gee|wiji)\\b[.!?,]*\\s*",
            RegexOption.IGNORE_CASE
        )
    }

    private val lock = Mutex()
    private var awakeSince: Long? = null
    private var busyUntil = 0L
    // text since the wake word that has not yet formed a question
    private var heardSinceWake = ""
    // the latest sentence ending in "?", waiting out the settle time
    private var pendingQuestion: String? = null
    private var settleJob: Job? = null

    suspend fun heard(utterance: String) = lock.withLock {
        val t = now()

        if (t < busyUntil) {
            logger.debug { "bot is busy; ignoring \"$utterance\"" }
            return@withLock
        }

        var text = utterance.trim()
        val wake = WAKE_WORD.find(text)
        if (wake != null) {
            // the wake word restarts the question, whatever was said before it
            if (!isAwake(t)) bot.heyOuija()
            awakeSince = t
            heardSinceWake = ""
            pendingQuestion = null
            settleJob?.cancel()
            text = text.substring(wake.range.last + 1).trim()
        }

        if (!isAwake(t)) {
            logger.debug { "asleep; ignoring \"$utterance\"" }
            return@withLock
        }
        if (text.isEmpty()) return@withLock

        heardSinceWake = (heardSinceWake + " " + text).trim()
        if (heardSinceWake.endsWith("?")) {
            // wait a moment: more of the question may follow, and if another "?" arrives this timer is replaced.
            // Text that does not end in "?" (a trailing "Thank you.") leaves the pending question alone.
            pendingQuestion = lastSentence(heardSinceWake)
            settleJob?.cancel()
            settleJob = scope.launch {
                delay(timings.settleMs)
                lock.withLock { pendingQuestion?.let { ask(it) } }
            }
        }
    }

    /** The Nano reports that the bot has finished spelling, so the next question can be taken. */
    suspend fun botIdle() = lock.withLock {
        busyUntil = 0
    }

    private fun isAwake(t: Long) = awakeSince?.let { t - it <= timings.wakeWindowMs } == true

    private suspend fun ask(question: String) {
        // the answer uses up the wake word, so a follow-up question needs a new one
        awakeSince = null
        heardSinceWake = ""
        pendingQuestion = null

        val answer = answerFor(question)?.lowercase()?.filter { it in 'a'..'z' || it in '0'..'9' }?.ifEmpty { null }
        val accepted: Boolean
        val busyMs: Long
        if (answer == null) {
            logger.debug { "for question \"$question\", sending Try Again" }
            accepted = bot.tryAgain()
            busyMs = timings.minBusyMs
        } else {
            logger.debug { "for question \"$question\", sending answer \"$answer\"" }
            accepted = bot.answer(answer)
            busyMs = maxOf(timings.minBusyMs, if (answer == "yes" || answer == "no") 0 else answer.length * timings.msPerLetter)
        }
        if (accepted) busyUntil = now() + busyMs
    }

    // "Hey, Ouija. It is late. Will I win?" -> "Will I win?"
    private fun lastSentence(s: String): String {
        val previousPunctuation = s.removeSuffix("?").lastIndexOfAny(charArrayOf('.', '!', '?'))
        return s.substring(previousPunctuation + 1).trim()
    }
}
