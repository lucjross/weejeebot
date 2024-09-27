package com.lucjross.ouija.daemon

import com.lucjross.ouija.QuestionTypeNode
import com.lucjross.ouija.client.BotClient
import com.lucjross.ouija.client.PalmClient
import mu.KotlinLogging
import org.springframework.stereotype.Component
import java.time.Clock
import java.time.Instant
import java.util.concurrent.*


@Component
class OuijaState(
    private val botClient: BotClient,
    private val scheduledExecutorService: ScheduledExecutorService,
    private val palmClient: PalmClient) {

    private val logger = KotlinLogging.logger {}

    private var heardHeyOuija = Instant.EPOCH
    private var answerSent = Instant.EPOCH
    private val clock = Clock.systemDefaultZone()
    // entries ordered by length descending
    private val debouncedAnswers: ConcurrentNavigableMap<String, ScheduledFuture<*>> =
        ConcurrentSkipListMap { o1, o2 -> o1.length - o2.length }

    @Synchronized
    fun heard(transcribed: String) {
        val heyOuijaMatch = Regex("Hey,? Ouija[.!]\\s*").find(transcribed)
        if (heyOuijaMatch != null) {
            if (heardHeyOuija.isBefore(clock.instant().minusSeconds(10))) {
                botClient.sendHeyOuija();

                // this can cancel any running answer
                answerSent = Instant.EPOCH
            }

            heardHeyOuija = clock.instant();
        }

        if (answerSent.isBefore(clock.instant().minusSeconds(5))) {

            var strWithoutHeyOuija = heyOuijaMatch?.let { transcribed.removePrefix(it.value) } ?: transcribed;

            if (strWithoutHeyOuija.endsWith("?") && heardHeyOuija.isAfter(clock.instant().minusSeconds(15))) {

                val previousPuncIdx = strWithoutHeyOuija.removeSuffix("?").lastIndexOfAny(charArrayOf('.', '!', '?')) + 1
                if (previousPuncIdx > 0) {
                    strWithoutHeyOuija = strWithoutHeyOuija.substring(previousPuncIdx).trim()
                }

                // debounce by removing previous entries (shorter or same questions) from the scheduled tasks
                var removed: String?
                do {
                    removed = debouncedAnswers.floorKey(strWithoutHeyOuija)?.apply { debouncedAnswers.remove(this) }
                    removed?.apply { logger.debug("debounced answer: $this") }
                } while (removed != null)

                val future = scheduledExecutorService.schedule({
                    if (debouncedAnswers.containsKey(strWithoutHeyOuija)) {
                        // occasionally multiple complete questions end up in the queue,
                        // like when it gives two alternatives for an ambiguous word,
                        // so empty the queue upon sending an answer
                        debouncedAnswers.clear()

                        var answer = palmClient.ask(strWithoutHeyOuija)
                        if (answer == null) {
                            val answerTypes = QuestionTypeNode.match(strWithoutHeyOuija)
                            answer = answerTypes.random().answer.invoke()
                        }

                        if (answer == null) {
                            logger.debug { "For question \"$strWithoutHeyOuija\", sending Try Again" }
                            botClient.sendTryAgain()
                        } else {
                            logger.debug { "For question \"$strWithoutHeyOuija\" sending answer \"$answer\"" }
                            botClient.sendAnswer(answer)
                        }

                        answerSent = clock.instant()
                    }
                }, 1, TimeUnit.SECONDS)
                debouncedAnswers[strWithoutHeyOuija] = future
            }
        }
    }
}

