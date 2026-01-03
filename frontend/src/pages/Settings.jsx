import React, { useState } from 'react'
import { motion } from 'framer-motion'
import toast from 'react-hot-toast'
import {
    Wifi, Database, Bot, Sliders, Bell, Shield, Monitor,
    Save, RefreshCw, AlertCircle
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'

const SettingSection = ({ title, description, icon: Icon, children }) => (
    <div className="bg-gray-900 border border-gray-800 rounded-xl p-6 mb-6">
        <div className="flex items-start gap-4 mb-6">
            <div className="p-3 bg-primary-500/10 rounded-lg">
                <Icon className="w-6 h-6 text-primary-400" />
            </div>
            <div className="flex-1">
                <h3 className="text-white font-semibold mb-1">{title}</h3>
                <p className="text-gray-400 text-sm">{description}</p>
            </div>
        </div>
        <div className="space-y-4">{children}</div>
    </div>
)

const SettingItem = ({ label, description, children }) => (
    <div className="flex items-start justify-between py-3 border-b border-gray-800 last:border-0">
        <div className="flex-1 mr-4">
            <label className="text-gray-300 font-medium block mb-1">{label}</label>
            {description && <p className="text-gray-500 text-sm">{description}</p>}
        </div>
        <div className="flex items-center">{children}</div>
    </div>
)

const Switch = ({ checked, onChange }) => (
    <button
        onClick={onChange}
        className={`relative w-12 h-6 rounded-full transition-colors ${
            checked ? 'bg-primary-600' : 'bg-gray-700'
        }`}
    >
        <motion.div
            animate={{ x: checked ? 24 : 2 }}
            className="absolute top-1 w-4 h-4 bg-white rounded-full"
        />
    </button>
)

const DEFAULTS = {
    rosbridge_url: 'ws://localhost:9091',
    ws_url: 'http://localhost:8082/ws/robot',
    video_stream_url: 'http://localhost:8082',
    auto_reconnect: true,
    reconnect_interval: 5000,
    default_model: 'burger',
    max_linear_velocity: 0.22,
    max_angular_velocity: 2.84,
    acceleration_limit: 0.1,
    auto_start_gazebo: true,
    use_gpu: false,
    physics_engine: 'ode',
    real_time_factor: 1.0,
    dark_mode: true,
    show_fps: true,
    show_grid: true,
    grid_size: 10,
    enable_notifications: true,
    sound_alerts: false,
    email_notifications: false,
}

export default function Settings() {
    const [settings, setSettings] = useState(() => {
        // localStorage’tan oku; yoksa DEFAULTS
        const saved = localStorage.getItem('app_settings')
        return saved ? { ...DEFAULTS, ...JSON.parse(saved) } : { ...DEFAULTS }
    })
    const [hasChanges, setHasChanges] = useState(false)

    const updateSetting = (key, value) => {
        setSettings(prev => ({ ...prev, [key]: value }))
        setHasChanges(true)
    }

    const saveSettings = () => {
        localStorage.setItem('app_settings', JSON.stringify(settings))
        // bazıları ayrı ayrı da dursun (boot için):
        localStorage.setItem('rosbridge_url', settings.rosbridge_url)
        localStorage.setItem('ws_url', settings.ws_url)
        toast.success('Settings saved successfully')
        setHasChanges(false)
    }

    const resetSettings = () => {
        setSettings({ ...DEFAULTS })
        setHasChanges(true)
        toast.success('Settings reset to defaults (remember to save)')
    }

    return (
        <PageContainer
            title="Settings"
            description="Configure your TurtleBot3 simulation environment"
            actions={
                <div className="flex items-center gap-3">
                    {hasChanges && (
                        <motion.div
                            initial={{ opacity: 0, scale: 0.9 }}
                            animate={{ opacity: 1, scale: 1 }}
                            className="flex items-center gap-2 px-3 py-1 bg-yellow-900/50 text-yellow-400 rounded-lg text-sm"
                        >
                            <AlertCircle className="w-4 h-4" />
                            Unsaved changes
                        </motion.div>
                    )}
                    <button
                        onClick={resetSettings}
                        className="px-4 py-2 bg-gray-800 hover:bg-gray-700 text-gray-300 rounded-lg transition-colors flex items-center gap-2"
                    >
                        <RefreshCw className="w-4 h-4" />
                        Reset
                    </button>
                    <button
                        onClick={saveSettings}
                        disabled={!hasChanges}
                        className="px-4 py-2 bg-primary-600 hover:bg-primary-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-colors flex items-center gap-2"
                    >
                        <Save className="w-4 h-4" />
                        Save Changes
                    </button>
                </div>
            }
        >
            {/* Connection Settings */}
            <SettingSection
                title="Connections"
                description="ROS Bridge, WebSocket ve video akış adresleri"
                icon={Wifi}
            >
                <SettingItem label="ROS Bridge URL" description="rosbridge_suite WebSocket endpoint">
                    <input
                        className="input w-72"
                        value={settings.rosbridge_url}
                        onChange={(e) => updateSetting('rosbridge_url', e.target.value)}
                    />
                </SettingItem>
                <SettingItem label="WebSocket URL" description="Backend STOMP/SockJS endpoint">
                    <input
                        className="input w-72"
                        value={settings.ws_url || ''}
                        onChange={(e) => updateSetting('ws_url', e.target.value)}
                    />
                </SettingItem>
                <SettingItem label="Video Stream URL" description="MJPEG server base URL">
                    <input
                        className="input w-72"
                        value={settings.video_stream_url}
                        onChange={(e) => updateSetting('video_stream_url', e.target.value)}
                    />
                </SettingItem>
                <SettingItem label="Auto Reconnect" description="Bağlantı koptuğunda otomatik bağlan">
                    <Switch
                        checked={settings.auto_reconnect}
                        onChange={() => updateSetting('auto_reconnect', !settings.auto_reconnect)}
                    />
                </SettingItem>
                <SettingItem label="Reconnect Interval (ms)">
                    <input
                        type="number"
                        className="input w-36"
                        value={settings.reconnect_interval}
                        onChange={(e) => updateSetting('reconnect_interval', Number(e.target.value))}
                        min={1000}
                        step={500}
                    />
                </SettingItem>
            </SettingSection>

            {/* Robot Settings */}
            <SettingSection
                title="Robot"
                description="Varsayılan model ve hız sınırları"
                icon={Bot}
            >
                <SettingItem label="Default Model">
                    <select
                        className="input w-48"
                        value={settings.default_model}
                        onChange={(e) => updateSetting('default_model', e.target.value)}
                    >
                        <option value="burger">TurtleBot3 Burger</option>
                        <option value="waffle">TurtleBot3 Waffle</option>
                        <option value="waffle_pi">TurtleBot3 Waffle Pi</option>
                    </select>
                </SettingItem>
                <SettingItem label="Max Linear Velocity (m/s)">
                    <input
                        type="number"
                        className="input w-36"
                        value={settings.max_linear_velocity}
                        onChange={(e) => updateSetting('max_linear_velocity', Number(e.target.value))}
                        min={0}
                        step={0.01}
                    />
                </SettingItem>
                <SettingItem label="Max Angular Velocity (rad/s)">
                    <input
                        type="number"
                        className="input w-36"
                        value={settings.max_angular_velocity}
                        onChange={(e) => updateSetting('max_angular_velocity', Number(e.target.value))}
                        min={0}
                        step={0.01}
                    />
                </SettingItem>
                <SettingItem label="Acceleration Limit">
                    <input
                        type="number"
                        className="input w-36"
                        value={settings.acceleration_limit}
                        onChange={(e) => updateSetting('acceleration_limit', Number(e.target.value))}
                        min={0}
                        step={0.01}
                    />
                </SettingItem>
            </SettingSection>

            {/* Simulation Settings */}
            <SettingSection
                title="Simulation"
                description="Gazebo, fizik motoru ve performans"
                icon={Sliders}
            >
                <SettingItem label="Auto Start Gazebo">
                    <Switch
                        checked={settings.auto_start_gazebo}
                        onChange={() => updateSetting('auto_start_gazebo', !settings.auto_start_gazebo)}
                    />
                </SettingItem>
                <SettingItem label="Use GPU">
                    <Switch
                        checked={settings.use_gpu}
                        onChange={() => updateSetting('use_gpu', !settings.use_gpu)}
                    />
                </SettingItem>
                <SettingItem label="Physics Engine">
                    <select
                        className="input w-48"
                        value={settings.physics_engine}
                        onChange={(e) => updateSetting('physics_engine', e.target.value)}
                    >
                        <option value="ode">ODE</option>
                        <option value="bullet">Bullet</option>
                        <option value="dart">DART</option>
                    </select>
                </SettingItem>
                <SettingItem label="Real Time Factor">
                    <input
                        type="number"
                        className="input w-36"
                        value={settings.real_time_factor}
                        onChange={(e) => updateSetting('real_time_factor', Number(e.target.value))}
                        min={0.1}
                        step={0.1}
                    />
                </SettingItem>
            </SettingSection>

            {/* UI Settings */}
            <SettingSection
                title="Interface"
                description="Görsel tercih ve grid ayarları"
                icon={Monitor}
            >
                <SettingItem label="Dark Mode">
                    <Switch
                        checked={settings.dark_mode}
                        onChange={() => updateSetting('dark_mode', !settings.dark_mode)}
                    />
                </SettingItem>
                <SettingItem label="Show FPS">
                    <Switch
                        checked={settings.show_fps}
                        onChange={() => updateSetting('show_fps', !settings.show_fps)}
                    />
                </SettingItem>
                <SettingItem label="Show Grid">
                    <Switch
                        checked={settings.show_grid}
                        onChange={() => updateSetting('show_grid', !settings.show_grid)}
                    />
                </SettingItem>
                <SettingItem label="Grid Size">
                    <input
                        type="number"
                        className="input w-28"
                        value={settings.grid_size}
                        onChange={(e) => updateSetting('grid_size', Number(e.target.value))}
                        min={1}
                        step={1}
                    />
                </SettingItem>
            </SettingSection>

            {/* Notifications */}
            <SettingSection
                title="Notifications"
                description="Bildirim tercihleri"
                icon={Bell}
            >
                <SettingItem label="Enable Notifications">
                    <Switch
                        checked={settings.enable_notifications}
                        onChange={() => updateSetting('enable_notifications', !settings.enable_notifications)}
                    />
                </SettingItem>
                <SettingItem label="Sound Alerts">
                    <Switch
                        checked={settings.sound_alerts}
                        onChange={() => updateSetting('sound_alerts', !settings.sound_alerts)}
                    />
                </SettingItem>
                <SettingItem label="Email Notifications">
                    <Switch
                        checked={settings.email_notifications}
                        onChange={() => updateSetting('email_notifications', !settings.email_notifications)}
                    />
                </SettingItem>
            </SettingSection>
        </PageContainer>
    )
}
