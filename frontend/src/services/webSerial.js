/**
 * WebSerial Service - Feetech STS3215 Binary Protocol Driver
 * 
 * HARDWARE: PC → USB-TTL Adapter → STS3215 Serial Bus Servo (NO Arduino)
 * PROTOCOL: Feetech SCS Smart Servo Binary Packets
 * BAUD RATE: 1,000,000 (1 Mbps) - Feetech default
 * 
 * Packet Structure:
 *   [0xFF][0xFF][ID][Length][Instruction][Param1]...[ParamN][Checksum]
 */

// SCS Protocol Constants
const SCS = {
    HEADER: [0xFF, 0xFF],

    // Instructions
    INST_PING: 0x01,
    INST_READ: 0x02,
    INST_WRITE: 0x03,
    INST_REG_WRITE: 0x04,
    INST_ACTION: 0x05,
    INST_SYNC_WRITE: 0x83,

    // Memory Addresses (STS3215)
    ADDR_TORQUE_ENABLE: 0x28,      // 40
    ADDR_GOAL_POSITION: 0x2A,      // 42 - Target position (2 bytes)
    ADDR_GOAL_TIME: 0x2C,          // 44 - Move time (2 bytes)
    ADDR_GOAL_SPEED: 0x2E,         // 46 - Move speed (2 bytes)
    ADDR_PRESENT_POSITION: 0x38,   // 56 - Current position (2 bytes)

    // Position Range
    MIN_POSITION: 0,
    MAX_POSITION: 4095,
    CENTER_POSITION: 2048,

    // Default servo ID
    DEFAULT_ID: 1,

    // Broadcast ID (all servos)
    BROADCAST_ID: 0xFE
};

export const webSerialService = {
    port: null,
    writer: null,
    reader: null,
    isConnected: false,
    _writableStreamClosed: null,
    _readableStreamClosed: null,

    isSupported() {
        return 'serial' in navigator;
    },

    /**
     * Connect to serial port with Feetech STS3215 configuration
     * Default baud rate: 1,000,000 (1 Mbps)
     */
    async connect(baudRate = 1000000) {
        if (!this.isSupported()) {
            throw new Error('Web Serial API not supported in this browser.');
        }

        try {
            this.port = await navigator.serial.requestPort();

            // STS3215 Serial Configuration: 8N1, 1Mbps
            await this.port.open({
                baudRate: baudRate,
                dataBits: 8,
                stopBits: 1,
                parity: 'none',
                flowControl: 'none'
            });

            console.log(`[STS3215] Port opened: ${baudRate} baud, 8N1`);

            // Setup binary writer (raw bytes, not text)
            this.writer = this.port.writable.getWriter();

            this.isConnected = true;
            console.log('[STS3215] ✅ Connection established');

            return { ok: true, portName: 'STS3215 Servo Bus' };
        } catch (error) {
            this.isConnected = false;
            console.error('[STS3215] ❌ Connection failed:', error);
            throw error;
        }
    },

    /**
     * Calculate SCS protocol checksum
     * Formula: ~(ID + Length + Instruction + Params...) & 0xFF
     */
    calculateChecksum(id, length, instruction, params) {
        let sum = id + length + instruction;
        for (const p of params) {
            sum += p;
        }
        return (~sum) & 0xFF;
    },

    /**
     * Build a complete SCS packet
     * @returns {Uint8Array} Complete packet with header and checksum
     */
    buildPacket(id, instruction, params = []) {
        const length = params.length + 2; // params + instruction + checksum
        const checksum = this.calculateChecksum(id, length, instruction, params);

        const packet = new Uint8Array([
            ...SCS.HEADER,      // 0xFF, 0xFF
            id,                 // Servo ID
            length,             // Length
            instruction,        // Instruction
            ...params,          // Parameters
            checksum            // Checksum
        ]);

        return packet;
    },

    /**
     * Send raw bytes to serial port
     * @param {Uint8Array} bytes - Binary data to send
     */
    async sendBytes(bytes) {
        if (!this.isConnected || !this.writer) {
            throw new Error('STS3215 not connected');
        }

        // Debug: Show packet bytes
        const hexString = Array.from(bytes).map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
        console.log(`[STS3215 TX] ${hexString}`);

        await this.writer.write(bytes);
    },

    /**
     * Set servo position (single servo)
     * @param {number} id - Servo ID (1-253)
     * @param {number} position - Target position (0-4095)
     * @param {number} time - Move time in ms (optional, 0 = max speed)
     */
    async setPosition(id, position, time = 0) {
        // Clamp position to valid range
        position = Math.max(SCS.MIN_POSITION, Math.min(SCS.MAX_POSITION, Math.floor(position)));
        time = Math.max(0, Math.min(65535, Math.floor(time)));

        // Split 16-bit values into low/high bytes
        const posLow = position & 0xFF;
        const posHigh = (position >> 8) & 0xFF;
        const timeLow = time & 0xFF;
        const timeHigh = (time >> 8) & 0xFF;

        // Build write packet: Address + Position(2) + Time(2)
        const params = [
            SCS.ADDR_GOAL_POSITION,  // Start address
            posLow, posHigh,          // Target position
            timeLow, timeHigh         // Move time
        ];

        const packet = this.buildPacket(id, SCS.INST_WRITE, params);
        await this.sendBytes(packet);

        console.log(`[STS3215] ID:${id} → Position:${position} (Time:${time}ms)`);
    },

    /**
     * Set multiple servo positions (sync write - all servos move simultaneously)
     * @param {Array<{id: number, position: number}>} servos - Array of servo commands
     * @param {number} time - Move time in ms for all servos
     */
    async syncWritePositions(servos, time = 500) {
        if (servos.length === 0) return;

        time = Math.max(0, Math.min(65535, Math.floor(time)));
        const timeLow = time & 0xFF;
        const timeHigh = (time >> 8) & 0xFF;

        // Sync write data length per servo: Position(2) + Time(2) = 4 bytes
        const dataLength = 4;

        // Build params: Address + DataLength + [ID + Data]...
        const params = [
            SCS.ADDR_GOAL_POSITION,  // Start address
            dataLength               // Data length per servo
        ];

        for (const servo of servos) {
            const position = Math.max(SCS.MIN_POSITION, Math.min(SCS.MAX_POSITION, Math.floor(servo.position)));
            const posLow = position & 0xFF;
            const posHigh = (position >> 8) & 0xFF;

            params.push(
                servo.id,
                posLow, posHigh,
                timeLow, timeHigh
            );
        }

        const packet = this.buildPacket(SCS.BROADCAST_ID, SCS.INST_SYNC_WRITE, params);
        await this.sendBytes(packet);

        console.log(`[STS3215] Sync write to ${servos.length} servos (Time:${time}ms)`);
    },

    /**
     * Enable/disable torque on a servo
     */
    async setTorque(id, enable) {
        const params = [
            SCS.ADDR_TORQUE_ENABLE,
            enable ? 1 : 0
        ];

        const packet = this.buildPacket(id, SCS.INST_WRITE, params);
        await this.sendBytes(packet);

        console.log(`[STS3215] ID:${id} Torque: ${enable ? 'ON' : 'OFF'}`);
    },

    /**
     * Ping a servo to check if connected
     */
    async ping(id) {
        const packet = this.buildPacket(id, SCS.INST_PING, []);
        await this.sendBytes(packet);
        console.log(`[STS3215] PING ID:${id}`);
    },

    /**
     * Convert degrees (0-180) to servo steps (0-4095)
     * @param {number} degrees - Angle in degrees
     * @param {number} minDeg - Minimum degrees (default -180)
     * @param {number} maxDeg - Maximum degrees (default 180)
     * @returns {number} Servo position (0-4095)
     */
    degreesToSteps(degrees, minDeg = -180, maxDeg = 180) {
        // Map from degree range to step range
        const range = maxDeg - minDeg;
        const normalized = (degrees - minDeg) / range;
        return Math.floor(normalized * SCS.MAX_POSITION);
    },

    /**
     * Kickstart test - send servo 1 to center position
     */
    async sendHomeTest() {
        console.log('[STS3215] 🏠 Sending HOME TEST...');
        await this.setPosition(1, SCS.CENTER_POSITION, 1000);
        console.log('[STS3215] 🏠 HOME TEST sent! (ID:1 → Center:2048)');
    },

    /**
     * Legacy ASCII send (for compatibility - not used with STS3215)
     */
    async send(dataString) {
        console.warn('[STS3215] ASCII send() called - use setPosition() for binary protocol');
        // Convert string to bytes and send anyway for debugging
        const encoder = new TextEncoder();
        const bytes = encoder.encode(dataString);
        await this.sendBytes(bytes);
    },

    async disconnect() {
        if (!this.port) return;

        try {
            if (this.writer) {
                this.writer.releaseLock();
                this.writer = null;
            }

            await this.port.close();
            this.port = null;
            this.isConnected = false;
            console.log('[STS3215] Disconnected');
        } catch (error) {
            console.warn('[STS3215] Disconnect warning:', error);
            this.isConnected = false;
            this.port = null;
            this.writer = null;
        }
    }
};

// Export protocol constants for external use
export { SCS };
