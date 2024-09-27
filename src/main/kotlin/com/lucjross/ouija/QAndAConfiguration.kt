package com.lucjross.ouija

import com.google.ai.generativelanguage.v1beta3.TextServiceClient
import com.google.ai.generativelanguage.v1beta3.TextServiceSettings
import com.google.api.gax.core.FixedCredentialsProvider
import com.google.api.gax.grpc.InstantiatingGrpcChannelProvider
import com.google.api.gax.rpc.FixedHeaderProvider
import io.awspring.cloud.autoconfigure.context.properties.AwsCredentialsProperties
import org.springframework.boot.context.properties.ConfigurationProperties
import org.springframework.boot.context.properties.ConstructorBinding
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration
import org.springframework.stereotype.Component
import org.springframework.web.reactive.function.client.WebClient
import software.amazon.awssdk.auth.credentials.DefaultCredentialsProvider
import software.amazon.awssdk.regions.Region
import software.amazon.awssdk.services.transcribestreaming.TranscribeStreamingAsyncClient
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import javax.sound.sampled.AudioInputStream

@Configuration
class QAndAConfiguration {

    @Bean
    fun transcribeStreamingAsyncClient(awsCredentialsProperties: AwsCredentialsProperties): TranscribeStreamingAsyncClient =
        TranscribeStreamingAsyncClient.builder()
            .credentialsProvider(
                DefaultCredentialsProvider.builder()
                    .profileName(awsCredentialsProperties.profileName)
                    .build()
            )
            .region(Region.US_WEST_2)
            .build()

    @Bean
    fun nanoClient(): WebClient {
        return WebClient.create("http://192.168.68.106");
    }

    @Bean
    fun scheduledExecutorService(): ScheduledExecutorService {
        return Executors.newSingleThreadScheduledExecutor()
    }

    @Bean
    fun audioInputStream(mic: Mic): AudioInputStream {
        return AudioInputStream(mic.getMic())
    }

    @Bean(destroyMethod = "destroy")
    fun audioStreamPublisher(audioInputStream: AudioInputStream): AudioStreamPublisher {
        return AudioStreamPublisher(audioInputStream)
    }

    @Bean
    fun textServiceClient(appProperties: AppProperties): TextServiceClient {
        val headers = mapOf("x-goog-api-key" to appProperties.palmApiKey)
        val provider = InstantiatingGrpcChannelProvider.newBuilder()
            .setHeaderProvider(FixedHeaderProvider.create(headers))
            .build()
        val settings = TextServiceSettings.newBuilder()
            .setTransportChannelProvider(provider)
            .setCredentialsProvider(FixedCredentialsProvider.create(null))
            .build()
        return TextServiceClient.create(settings)
    }
}

@ConfigurationProperties("app")
@ConstructorBinding
data class AppProperties(
    val palmApiKey: String
)