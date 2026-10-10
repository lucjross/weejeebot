# Backend: speech-to-text replacement (handoff)

Handoff notes for an agent continuing the analysis of how to replace Amazon Transcribe in the Kotlin backend.
Nothing here has been run or changed. Everything below was read from the source, and a few claims are marked
**VERIFY** because they come from memory or inference.

## Goal

The backend uses Amazon Transcribe streaming, which the owner finds expensive. It runs on the owner's laptop and
records users' voices through the laptop's onboard microphone. The task is to find and compare cheaper or free
replacements, recommend one, and (if asked) implement it. The owner has not yet said how many hours per month it runs,
what the venue is like, or what the laptop's hardware is (GPU or not). Ask before benchmarking.

## What the project is

A motorized Ouija planchette. A person says "Hey, Ouija" and asks a question. The backend decides on a one-word
answer and tells a Nano 33 IoT on the planchette to spell it by driving a bot over a custom board.
Repo layout:
- `src/` and `build.gradle.kts`: this Kotlin / Spring Boot backend (Boot 2.7.4, Kotlin 1.6.21, Java 17).
- `arduino/`: the microcontroller side (Uno on a Zumo bot, plus the Nano with display and WiFi). See `arduino/WIRING.md`.
  Those sketches are being rebuilt separately and are out of scope here.

## Current data flow

```
laptop mic (javax.sound TargetDataLine, 16 kHz, 16-bit, mono, little-endian)
  -> AudioInputStream  (QAndAConfiguration.kt)
  -> AudioStreamPublisher, 1024-byte chunks  (Audio.kt)
  -> TranscribeStreamingAsyncClient, en-US, PCM 16 kHz, custom vocabulary "Ouija01"  (TranscriptionDaemon.kt)
  -> every alternative of every result -> OuijaState.heard(String)
  -> wake word + question detection, 1 s debounce  (daemon/OuijaState.kt)
  -> PalmClient.ask(question)   [Google PaLM text-bison-001]
       or, if that returns null: QuestionTypeNode.match() -> random word from a list  (QandA.kt)
  -> BotClient POSTs to the Nano  (client/BotClient.kt)
```

## Files that matter

| File | Role |
|------|------|
| `src/main/kotlin/com/lucjross/ouija/daemon/TranscriptionDaemon.kt` | Starts a thread that opens the Transcribe stream and forwards transcripts. The only Transcribe-specific logic. |
| `src/main/kotlin/com/lucjross/ouija/Audio.kt` | `Mic` (opens the mic once, in a companion object) and `AudioStreamPublisher` (a reactive-streams bridge into the Transcribe SDK). |
| `src/main/kotlin/com/lucjross/ouija/QAndAConfiguration.kt` | Spring beans: Transcribe client, mic stream, Nano `WebClient`, PaLM client. |
| `src/main/kotlin/com/lucjross/ouija/daemon/OuijaState.kt` | The wake-word / question state machine. Its input contract is the key constraint for any replacement. |
| `src/main/kotlin/com/lucjross/ouija/QandA.kt` | Regex tree that classifies a question and picks a canned answer when PaLM gives nothing. |
| `src/main/kotlin/com/lucjross/ouija/client/PalmClient.kt` | Calls Google's text-bison-001 with a "pretend you're a psychic, one word under ten characters" prompt. |
| `src/main/kotlin/com/lucjross/ouija/client/BotClient.kt` | Sends HTTP POSTs to the Nano. |
| `src/main/resources/PhraseSoundsLikeIPADisplayAs.txt` | The custom-vocabulary source: "Ouija" sounds like "wee-jee", plus the name "Kadie". |
| `build.gradle.kts` | Dependencies (AWS SDK, spring-cloud-aws 2.3.0, a locally built Google gapic snapshot, jython-slim). |

## The contract any replacement must satisfy

`OuijaState.heard(transcribed: String)` expects text that looks like Transcribe output:
- **Punctuation and capitalization.** The wake word is matched with the regex `Hey,? Ouija[.!]\s*` (OuijaState.kt line 30),
  and a question is only accepted if the text **ends with `?`** (line 46). A recognizer that doesn't punctuate
  (Vosk, for example) would never trigger the state machine as written. The fix would be separate wake-word detection and a
  different way of deciding the question has ended, not just swapping the string source.
- **Partial results.** `TranscriptionDaemon.processTranscript` does not check `isPartial`. It calls `heard()` for every alternative
  of every result, partial or final, as Transcribe refines the text. The debounce in `OuijaState` (a map of pending answers
  keyed by question text, cancelling shorter ones, with a 1 s delay) exists to cope with that repetition. A replacement that
  emits only final results would simplify this, but the state machine would need re-tuning.
- **Timing.** After a wake word, a question is accepted for 15 s, and a new answer is not sent for 5 s after the last one.
  The wake word itself can cancel a running answer.
- **The wake word is "Hey, Ouija".** "Ouija" is pronounced "wee-jee", which general speech models often get wrong.
  This is the reason the custom vocabulary exists. Any replacement needs a way to bias toward it, such as
  hotwords, an initial prompt, a keyword spotter, or a post-processing fuzzy match.

## The Nano interface (what the backend must keep calling)

Hardcoded base URL `http://192.168.68.106` (QAndAConfiguration.kt).
- `POST /heyOuija`: show "?" on the display.
- `POST /answer` with form field `answer`: lowercase `a-z0-9`, or `yes` / `no`. Other characters are ignored.
  The Nano sketch checks that exactly two request args arrive (an odd quirk noted in the sketch).
- `POST /tryAgain`: show a red "?".

## Cost: where the money probably goes

**VERIFY current pricing.** The usual model for streaming speech services, including Transcribe streaming, is billing per second
of audio streamed. The backend streams the microphone **continuously for as long as it runs** and never pauses, so cost scales with
uptime, not with how much people actually speak. That points to a cheap first step that may not need a new vendor at all:
- Add voice-activity detection (VAD) so audio is only sent while someone is speaking.
- Or, run a small local wake-word detector for "Hey, Ouija" and stream audio only for the next ~15 s (the same window
  `OuijaState` already uses).
Either one could cut cost sharply even if the speech recognizer stays the same.

## Candidate replacements to evaluate

Not benchmarked. This is a starting list, not a recommendation.

1. **Local Whisper-family model** (whisper.cpp, faster-whisper) with a VAD in front (for example Silero VAD or webrtcvad).
   Free to run and punctuates, which suits the `heard()` contract. It is not natively streaming, so audio would be cut into
   speech segments, which adds latency. A hotword or initial prompt can bias "Ouija". From Kotlin, the likely options are a
   sidecar process or HTTP server in Python, a whisper.cpp server, or a JNI wrapper. CPU speed on the owner's laptop is the open question.
2. **Vosk (Kaldi)**, local and streaming, with JVM bindings. No punctuation, so it needs the wake-word rework above, and
   accuracy is lower than Whisper.
3. **Other cloud speech APIs** with free tiers or lower prices (Google Cloud Speech-to-Text, Azure Speech, Deepgram, AssemblyAI,
   OpenAI's transcription endpoints). **VERIFY** current pricing and free tiers. Batching short post-wake-word clips costs far less than continuous streaming.
4. **Browser Web Speech API** in a page on the laptop or a phone. Free, but it needs a browser tab and the quality varies by browser.
5. **Hybrid:** a local wake-word detector (openWakeWord, Porcupine, or a Vosk keyword grammar) plus a short transcription of
   only the question, using option 1 or 3.

Suggested evaluation criteria: monthly cost at the owner's real usage, end-to-end delay from the end of the question to the
Nano's reaction (a few seconds is the target), accuracy on "Hey, Ouija" and on question text in a noisy room, whether it
works offline at the venue, and how hard it is to integrate on the JVM.

## Suggested integration seam

Only `TranscriptionDaemon` and `Audio.kt` are Transcribe-specific. Put a small interface in front of them (for example
`SpeechRecognizer`, emitting strings to `OuijaState.heard`) so recognizers can be swapped and compared without touching the state machine.
Decide first whether the new recognizer emits partial results, and keep punctuation and the `?` ending, or rework `OuijaState`.

## Other findings from reading the code

These are outside the transcription question, but the next agent should know them.
- **PaLM is probably dead. VERIFY.** `PalmClient` calls `models/text-bison-001` through the PaLM / Generative Language `v1beta3` API, which Google
  has retired. The dependency `gapic-google-cloud-ai-generativelanguage-v1beta3-java:0.0.0-SNAPSHOT` is a locally built snapshot resolved from
  `mavenLocal()`, which makes the build fragile on any other machine.
- **A failing PaLM call silently produces no answer.** `PalmClient.ask` ends in `.block()`, which rethrows on error. The caller in
  `OuijaState.heard` doesn't catch it, so the exception ends the scheduled task, and neither an answer nor `sendTryAgain()` is sent. The canned-answer fallback only
  runs when `ask` returns null, not when it throws.
- **No reconnect.** When the Transcribe stream ends, `TranscriptionDaemon` logs "stream transcription done" and the daemon stops.
- **Config:** the AWS profile is `ouija` in region `us-west-2` (application.yaml). The PaLM key is the property `app.palm-api-key`,
  which is not in `application.yaml` and must be supplied some other way. No secrets are in the repo that I saw. Do not add any.
- **Dependencies look stale:** spring-cloud-aws 2.3.0, AWS SDK BOM 2.16.60, Boot 2.7.4, and `jython-slim` appears unused.
- **Language data in `QandA.kt`:** a profanity check (`AnswerType.BAD`) and themed word lists for the canned answers.

## Open questions for the owner

1. How many hours a month does the service run, and is it only during events?
2. Laptop CPU and GPU (this decides whether local Whisper is realistic)?
3. Does the venue have reliable internet, or does it need to work offline?
4. Is the PaLM replacement (a different LLM, or only the canned answers) in scope for this pass?
5. Is Transcribe's accuracy on the owner's voices and accents acceptable, and what is the bar for the replacement?
