/**
 * API client for MoveIt commands.
 * Connects to the backend bridge to send motion planning and execution commands.
 */
export async function sendMoveItCmd(cmd) {
    console.log(`MOVEIT_CMD_SENT cmd=${cmd}`);

    try {
        const response = await fetch("/api/moveit/cmd", {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
            },
            body: JSON.stringify({ cmd }),
        });

        if (!response.ok) {
            const errorData = await response.json().catch(() => ({}));
            throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        console.log(`MOVEIT_CMD_OK cmd=${cmd}`, data);
        return data;
    } catch (error) {
        console.error(`MOVEIT_CMD_ERR cmd=${cmd}`, error);
        throw error;
    }
}
