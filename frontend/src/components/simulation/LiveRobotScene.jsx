
import React, { useState, useEffect, useRef } from 'react';
import { Canvas, useLoader, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, ContactShadows } from '@react-three/drei';
import URDFLoader from 'urdf-loader';
import { LoadingManager, Color } from 'three';
import { degToRad } from 'three/src/math/MathUtils';

/**
 * URDF JOINT MAPPING - "Rosetta Stone"
 * Maps React UI state keys → Actual URDF joint names
 * 
 * URDF Joint Names (from so_arm101.urdf):
 *   - shoulder_pan   (base rotation, z-axis)
 *   - shoulder_lift  (shoulder pitch, z-axis)
 *   - elbow_flex     (elbow bend, z-axis)
 *   - wrist_flex     (wrist pitch, z-axis)
 *   - wrist_roll     (wrist rotation, z-axis)
 *   - gripper        (jaw open/close, z-axis)
 */
const URDF_JOINT_MAP = {
    // UI State Key → URDF Joint Name
    base: 'shoulder_pan',
    shoulder: 'shoulder_lift',
    elbow: 'elbow_flex',
    wristPitch: 'wrist_flex',
    wristRoll: 'wrist_roll',
    gripper: 'gripper'
};

const RobotModel = ({ joints }) => {
    // 1. Create a Manager to rewrite paths
    const [manager] = useState(() => {
        const m = new LoadingManager();
        m.setURLModifier((url) => {
            // Generic Handler: 'package://<package_name>/meshes/...' -> '/urdf/meshes/...'
            if (url.startsWith('package://')) {
                // Remove 'package://' and then the first folder segment (package name)
                const parts = url.replace('package://', '').split('/');
                parts.shift(); // Remove package name
                return `/urdf/${parts.join('/')}`;
            }

            // Fallback for relative paths
            if (!url.startsWith('/') && !url.startsWith('http') && !url.startsWith('blob:')) {
                return `/urdf/${url}`;
            }
            return url;
        });
        return m;
    });

    // 2. Load the Model
    const robot = useLoader(URDFLoader, '/urdf/so_arm101.urdf', (loader) => {
        loader.manager = manager;
    });

    // 3. Material Override + Joint Discovery Debug
    const hasLoggedJoints = useRef(false);

    useEffect(() => {
        if (robot) {
            // Debug: Log all available URDF joints once
            if (!hasLoggedJoints.current && robot.joints) {
                hasLoggedJoints.current = true;
                const jointNames = Object.keys(robot.joints);
                console.log('[URDF DEBUG] Discovered Joints:', jointNames);
                console.log('[URDF DEBUG] Joint Objects:', robot.joints);

                // Verify mapping exists
                Object.entries(URDF_JOINT_MAP).forEach(([uiKey, urdfName]) => {
                    const exists = robot.joints[urdfName] !== undefined;
                    console.log(`[URDF DEBUG] Mapping: ${uiKey} → ${urdfName}: ${exists ? '✅ FOUND' : '❌ MISSING'}`);
                });
            }

            // Apply material
            robot.traverse((child) => {
                if (child.isMesh) {
                    child.material.color = new Color('#2ecc71'); // Bambot Green
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
            });
        }
    }, [robot]);

    // 4. Joint Update - Using Correct URDF_JOINT_MAP
    useFrame(() => {
        if (!robot || !robot.joints) return;

        // Iterate over UI state keys and map to URDF joint names
        for (const [uiKey, urdfJointName] of Object.entries(URDF_JOINT_MAP)) {
            const joint = robot.joints[urdfJointName];
            if (!joint) continue;

            const val = joints[uiKey];
            if (val === undefined) continue;

            try {
                // Handle gripper as percentage (0-100) → mapped to joint limits
                if (urdfJointName === 'gripper') {
                    const lower = joint.limit?.lower ?? -0.174533;
                    const upper = joint.limit?.upper ?? 1.74533;
                    const range = upper - lower;
                    const clampedVal = Math.max(0, Math.min(100, val));
                    const target = lower + ((clampedVal / 100) * range);
                    joint.setJointValue(target);
                } else {
                    // All other joints: degrees → radians
                    joint.setJointValue(degToRad(val));
                }
            } catch (err) {
                // Silent failure to avoid console spam
            }
        }
    });

    // 5. Render (Rotate -90 X to fix Z-up orientation)
    return <primitive object={robot} rotation={[-Math.PI / 2, 0, 0]} />;
};

const LiveRobotScene = ({ joints }) => {
    return (
        <div className="h-full w-full bg-[#111111]">
            <Canvas shadows camera={{ position: [1, 1, 1], fov: 45 }}>
                <color attach="background" args={['#111111']} />

                {/* Lighting */}
                <ambientLight intensity={0.6} />
                <directionalLight
                    position={[5, 10, 5]}
                    intensity={1.5}
                    castShadow
                    shadow-mapSize={[2048, 2048]}
                />
                <spotLight position={[-2, 5, -2]} intensity={1} color="#00a8ff" />

                {/* Ground */}
                <Grid
                    infiniteGrid
                    cellSize={0.05}
                    sectionSize={0.5}
                    fadeDistance={3}
                    fadeStrength={1.5}
                    sectionColor="#444444"
                    cellColor="#222222"
                />
                <ContactShadows opacity={0.4} scale={10} blur={2} far={2} />

                {/* Controls */}
                <OrbitControls makeDefault minPolarAngle={0} maxPolarAngle={Math.PI / 1.8} target={[0, 0.2, 0]} />

                {/* Robot Model */}
                <RobotModel joints={joints} />
            </Canvas>
        </div>
    );
};

export default LiveRobotScene;


