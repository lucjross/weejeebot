package com.lucjross.ouija

enum class AnswerType(val answer: () -> String?) {
    VERB_CONTINUOUS({ listOf(
        "running",
        "skipping",
        "dancing",
        "dragging",
        "thinking",
        "golfing",
        "laughing",
        "sleeping",
        "shopping",
        "painting",
        "singing"
    ).random() }),
    VERB_PRESENT({ listOf(
        "sit",
        "swim",
        "read",
        "play",
        "cook",
        "sleep",
        "cook",
        "dance",
        "eat",
        "die"
    ).random() }),
    NOUN_SINGULAR({ listOf(
        "grave",
        "cauldron",
        "shoe",
        "zombie",
        "ghost",
        "pumpkin",
        "spider",
        "taco",
        "knife",
        "sword",
        "blood",
        "nothing"
    ).random() }),
    NAME({ listOf(
        "bob",
        "winifred",
        "sabrina",
        "eleven",
        "satan",
        "lucifer",
        "harrystyles",
        "jojo",
        "lizzo",
        "batman",
        "spiderman",
        "joebiden"
    ).random() }),
    PLACE({ listOf(
        "banff",
        "salem",
        "gotham",
        "springfield",
        "asgard",
        "hogwarts",
        "hawkins",
        "egypt",
        "hell",
        "austin",
        "texas",
        "cemetery"
    ).random() }),
    TIME({ listOf(
        "today",
        "yesterday",
        "tomorrow",
        "1982",
        "october",
        "1999",
        "1985",
        "1261",
        "tuesday",
        "thursday",
        "midnight"
    ).random() }),
    ADJECTIVE({ listOf(
        "spooky",
        "ugly",
        "hot",
        "cold",
        "sneaky",
        "bloody"
    ).random() }),
    //    VERB_NOUN(),
    NUMBER({ (0..99).random().toString() }),
    NONE({ null }),
    BAD({ null }),
    BINARY({ listOf("yes", "no").random() }),
    REASON({ listOf(
        "because",
        "idk",
        "toocool",
        "2legit",
        "sus"
    ).random() })
}

data class QuestionTypeNode(val regex: String?,
                            val questionTypeNodes: List<QuestionTypeNode> = listOf(),
                            val answerTypes: List<AnswerType> = listOf(AnswerType.NONE)) {
    companion object {
        private val tree =
            QuestionTypeNode(null, listOf(
                QuestionTypeNode("(fuck|bitch|shit|cunt|dick|pussy|nigg)", listOf(), listOf(AnswerType.BAD)),

                QuestionTypeNode("^(Is|Am|Are|Was|Were|Has|Have|Do|Does|Did|Can|Could|May|Will|Would|Should) ",
                    listOf(), listOf(AnswerType.BINARY)),

                QuestionTypeNode("^What('s)? ", listOf(
                    QuestionTypeNode("^(is|are|am) ", listOf(
                        QuestionTypeNode(" doing\\?", listOf(), listOf(AnswerType.VERB_CONTINUOUS)),

                        QuestionTypeNode(" do\\?", listOf(), listOf(AnswerType.VERB_PRESENT)),

                        QuestionTypeNode(" name\\?", listOf(), listOf(AnswerType.NAME))
                    ), listOf(AnswerType.NOUN_SINGULAR)),

                    QuestionTypeNode("^(did|do|does) ", listOf(
                        QuestionTypeNode(" do\\?", listOf(), listOf(AnswerType.VERB_PRESENT))
                    ), listOf(AnswerType.NOUN_SINGULAR)),

                    QuestionTypeNode("^(can|could|would|should) ", listOf(), listOf(AnswerType.VERB_PRESENT)),

                    QuestionTypeNode(" name( of|\\?)", listOf(), listOf(AnswerType.NAME))
                ), listOf(AnswerType.VERB_PRESENT)),

                QuestionTypeNode("^Who('s)? ", listOf(), listOf(AnswerType.NAME)),

                QuestionTypeNode("^Where('s)? ", listOf(), listOf(AnswerType.PLACE)),

                QuestionTypeNode("^How ", listOf(
                    QuestionTypeNode("^(many|old) ", listOf(), listOf(AnswerType.NUMBER))
                ), listOf(AnswerType.VERB_CONTINUOUS)),

                QuestionTypeNode("^When('s)? ", listOf(), listOf(AnswerType.TIME)),

                QuestionTypeNode("^Why ", listOf(), listOf(AnswerType.REASON))
            ))

        fun match(s: String): List<AnswerType> {
            val cleaned = Regex("^Hey, Ouija[.,!]\\s+").find(s)?.let { s.removePrefix(it.value) } ?: s
            return tree.match0(cleaned)
        }
    }

    private fun match0(s: String): List<AnswerType> {
        val matchResult = regex?.let { Regex(it, RegexOption.IGNORE_CASE) }?.find(s);
        return if (matchResult != null || regex == null) {
            val trimmed = s.removePrefix(matchResult?.value ?: "")
            val child = questionTypeNodes.find {
                it.match0(trimmed).any { answerType -> answerType == AnswerType.BAD }
            } ?: questionTypeNodes.firstOrNull {
                it.match0(trimmed).none { answerType -> answerType == AnswerType.NONE }
            }
            child?.match0(trimmed) ?: this.answerTypes
        } else {
            listOf(AnswerType.NONE)
        }
    }
}
