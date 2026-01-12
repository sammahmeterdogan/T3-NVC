package com.samma.rcp.app.orchestration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Map;

/**
 * Invokes docker/compose CLI commands.
 * Path-independent for CI/CD; compose file specified via "-f".
 */
@Component
public class DockerService {
    private static final Logger log = LoggerFactory.getLogger(DockerService.class);

    /** docker compose up -d with environment variables */
    public void composeUp(Path composeFile, Map<String, String> env) {
        run(env, "docker", "compose", "-f", composeFile.toString(), "up", "-d");
    }

    /** docker compose down */
    public void composeDown(Path composeFile) {
        run(null, "docker", "compose", "-f", composeFile.toString(), "down");
    }

    /** Checks if host:port is reachable within timeout */
    public boolean waitForPort(String host, int port, Duration timeout) {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (System.nanoTime() < deadline) {
            try (Socket s = new Socket()) {
                s.connect(new InetSocketAddress(host, port), 1000);
                return true;
            } catch (IOException ignored) {
                try { Thread.sleep(300); } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt(); 
                    return false;
                }
            }
        }
        return false;
    }

    private void run(Map<String, String> env, String... cmd) {
        try {
            ProcessBuilder pb = new ProcessBuilder(cmd).redirectErrorStream(true);
            if (env != null && !env.isEmpty()) {
                pb.environment().putAll(env);
            }
            Process p = pb.start();
            String out = new String(p.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            int code = p.waitFor();
            log.info("[compose] {}", out.trim());
            if (code != 0) {
                throw new IllegalStateException("Process exit code: " + code);
            }
        } catch (IOException | InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
        }
    }
}
