package com.samma.rcp.app.dto;

/**
 * Tek tip durum yanıtı: UI bu JSON’u bekler.
 * running  : sim açık mı?
 * wsUrl    : roslibjs için ws://HOST:PORT
 * port     : istendiğinde ayrı kullanmak için
 */
public class SimStatusDto {

    private boolean running;
    private String wsUrl;
    private int port;

    public SimStatusDto() {}

    public SimStatusDto(boolean running, String wsUrl, int port) {
        this.running = running;
        this.wsUrl = wsUrl;
        this.port = port;
    }

    public boolean isRunning() {
        return running;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    public String getWsUrl() {
        return wsUrl;
    }

    public void setWsUrl(String wsUrl) {
        this.wsUrl = wsUrl;
    }

    public int getPort() {
        return port;
    }

    public void setPort(int port) {
        this.port = port;
    }
}
