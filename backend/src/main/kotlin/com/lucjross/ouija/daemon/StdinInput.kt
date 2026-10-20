package com.lucjross.ouija.daemon

import io.github.oshai.kotlinlogging.KotlinLogging
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

/**
 * Stands in for the microphone: every line typed on stdin is handled as if it had been heard. A line with no ending
 * punctuation gets a "?" so it counts as a question: "hey ouija" and "will I win" work as two lines, or together as
 * "hey ouija will I win".
 */
class StdinInput(private val ouijaState: OuijaState, private val readLine: () -> String? = ::readlnOrNull) {

    private val logger = KotlinLogging.logger {}

    fun start(scope: CoroutineScope): Job = scope.launch(Dispatchers.IO) {
        logger.info { "reading questions from stdin: type \"hey ouija\", then a question, one line at a time" }
        while (isActive) {
            val line = readLine()?.trim() ?: break
            if (line.isEmpty()) continue
            val text = if (line.last() in ".?!") line else "$line?"
            logger.debug { "[stdin] $text" }
            ouijaState.heard(text)
        }
        logger.info { "stdin closed; no more typed input" }
    }
}
