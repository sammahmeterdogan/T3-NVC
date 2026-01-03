package com.samma.rcp.app.config;

import org.springframework.boot.context.properties.ConfigurationProperties;

@ConfigurationProperties(prefix = "ros")
public class RosProperties {

    private Docker docker = new Docker();
    private Bridge bridge = new Bridge();
    private Visualization visualization = new Visualization();

    public Docker getDocker() { return docker; }
    public Bridge getBridge() { return bridge; }
    public Visualization getVisualization() { return visualization; }

    public static class Docker {
        private String host;
        private String composeFile;
        private String network;

        public String getHost() { return host; }
        public void setHost(String host) { this.host = host; }

        public String getComposeFile() { return composeFile; }
        public void setComposeFile(String composeFile) { this.composeFile = composeFile; }

        public String getNetwork() { return network; }
        public void setNetwork(String network) { this.network = network; }
    }

    public static class Bridge {
        private String url;
        private long reconnectInterval;

        public String getUrl() { return url; }
        public void setUrl(String url) { this.url = url; }

        public long getReconnectInterval() { return reconnectInterval; }
        public void setReconnectInterval(long reconnectInterval) { this.reconnectInterval = reconnectInterval; }
    }

    public static class Visualization {
        private String rvizUrl;
        private String turtlesimUrl;

        public String getRvizUrl() { return rvizUrl; }
        public void setRvizUrl(String rvizUrl) { this.rvizUrl = rvizUrl; }

        public String getTurtlesimUrl() { return turtlesimUrl; }
        public void setTurtlesimUrl(String turtlesimUrl) { this.turtlesimUrl = turtlesimUrl; }
    }
}
