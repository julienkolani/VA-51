"""
TurtleBot WebSocket Bridge ROS2 
Bridge pur entre WebSocket et ROS2 cmd_vel
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import asyncio
import websockets
from threading import Thread, Lock
from geometry_msgs.msg import Twist
import math
import socket
from datetime import datetime
import traceback


class SafetyConfig:
    """Configuration des limites de vitesse"""
    def __init__(self):
        self.max_linear_vel = 0.5
        self.min_linear_vel = -0.3
        self.max_angular_vel = 1.5


class VelocityLimiter:
    """Limiteur simple de vitesse (clamp uniquement)"""
    
    def __init__(self, config, logger):
        self.config = config
        self.lock = Lock()
        self.logger = logger
        
    def validate_command(self, linear_x, angular_z):
        """Validation simple : clamp aux limites physiques"""
        with self.lock:
            original_linear = linear_x
            original_angular = angular_z
            
            # Clamp aux limites configurées
            linear_x = np.clip(linear_x, self.config.min_linear_vel, self.config.max_linear_vel)
            angular_z = np.clip(angular_z, -self.config.max_angular_vel, self.config.max_angular_vel)
            
            # Déterminer le statut
            if abs(linear_x) < 0.001 and abs(angular_z) < 0.001:
                reason = "Stop"
            elif abs(original_linear - linear_x) > 0.001 or abs(original_angular - angular_z) > 0.001:
                reason = "Speed Limited"
            else:
                reason = "OK"
            
            return True, linear_x, angular_z, reason


class ROSBridge(Node):
    """Pont simple entre WebSocket et ROS2"""
    
    def __init__(self):
        super().__init__('websocket_bridge')
        
        self._print_startup_banner()
        
        # Paramètres ROS2
        self._declare_and_get_parameters()
        
        # Configuration
        self.config = SafetyConfig()
        self.config.max_linear_vel = self.max_linear_vel
        self.config.min_linear_vel = self.min_linear_vel
        self.config.max_angular_vel = self.max_angular_vel
        
        self.velocity_limiter = VelocityLimiter(self.config, self.get_logger())
        
        self._print_configuration()
        
        # Publisher cmd_vel uniquement
        self._setup_ros_interfaces()
        
        # Stats
        self.commands_received = 0
        self.commands_accepted = 0
        self.commands_blocked = 0
        self.last_status = "Initialisation"
        self.connected_clients = set()
        self.start_time = self.get_clock().now()
        
        # WebSocket
        self.ws_server = None
        self.running = True
        
        self._print_network_info()
        
        self.get_logger().info("✅ [INIT] Bridge prêt à démarrer")
    
    def _print_startup_banner(self):
        """Affiche la bannière de démarrage"""
        banner = """
╔══════════════════════════════════════════════════════════════╗
║            _ TURTLEBOT WEBSOCKET BRIDGE ROS2 _               ║
╚══════════════════════════════════════════════════════════════╝
"""
        print(banner)
        self.get_logger().info("🚀 [INIT] Démarrage du WebSocket Bridge")
    
    def _declare_and_get_parameters(self):
        """Déclare et récupère tous les paramètres ROS2"""
        # Déclaration
        self.declare_parameter('robot_namespace', 'robot_controler')
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('min_linear_vel', -0.3)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('log_level', 'INFO')
        
        # Récupération
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.ws_host = self.get_parameter('ws_host').value
        self.ws_port = self.get_parameter('ws_port').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.log_level = self.get_parameter('log_level').value
        
        self.get_logger().info("⚙️  [INIT] Paramètres ROS2 chargés:")
        self.get_logger().info(f"   ├─ robot_namespace: {self.robot_namespace}")
        self.get_logger().info(f"   ├─ ws_host: {self.ws_host}")
        self.get_logger().info(f"   ├─ ws_port: {self.ws_port}")
        self.get_logger().info(f"   ├─ max_linear_vel: {self.max_linear_vel}")
        self.get_logger().info(f"   ├─ min_linear_vel: {self.min_linear_vel}")
        self.get_logger().info(f"   ├─ max_angular_vel: {self.max_angular_vel}")
        self.get_logger().info(f"   └─ log_level: {self.log_level}")
    
    def _setup_ros_interfaces(self):
        """Configure le publisher ROS2"""
        self.get_logger().info("🔧 [INIT] Configuration des interfaces ROS2...")
        
        # QoS Profile pour meilleure fiabilité
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher cmd_vel uniquement
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile
        )
        self.get_logger().info(f"   └─ ✅ Publisher: /{self.robot_namespace}/cmd_vel")
    
    def _print_configuration(self):
        """Affiche la configuration détaillée"""
        self.get_logger().info("╔" + "═"*58 + "╗")
        self.get_logger().info("║" + " "*18 + "CONFIGURATION" + " "*27 + "║")
        self.get_logger().info("╠" + "═"*58 + "╣")
        self.get_logger().info(f"║ 🤖 Robot: {self.robot_namespace:<44} ║")
        self.get_logger().info(f"║ 🌐 WebSocket: {self.ws_host}:{self.ws_port:<34} ║")
        self.get_logger().info(f"║ ⚡ Vitesse linéaire: [{self.min_linear_vel:>6.2f}, {self.max_linear_vel:>5.2f}] m/s" + " "*11 + "║")
        self.get_logger().info(f"║ 🔄 Vitesse angulaire: ±{self.max_angular_vel:.2f} rad/s" + " "*21 + "║")
        self.get_logger().info("╚" + "═"*58 + "╝")
    
    def _print_network_info(self):
        """Affiche les informations réseau"""
        try:
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            
            self.get_logger().info("╔" + "═"*58 + "╗")
            self.get_logger().info("║" + " "*17 + "INFORMATIONS RÉSEAU" + " "*21 + "║")
            self.get_logger().info("╠" + "═"*58 + "╣")
            self.get_logger().info(f"║ 🖥️  Hostname: {hostname:<42} ║")
            self.get_logger().info(f"║ 🌐 IP locale: {local_ip:<41} ║")
            self.get_logger().info(f"║ 🔌 Port WebSocket: {self.ws_port:<36} ║")
            self.get_logger().info(f"║ 📡 URL: ws://{local_ip}:{self.ws_port:<31} ║")
            if self.ws_host == '0.0.0.0':
                self.get_logger().info(f"║ ℹ️  Écoute sur toutes les interfaces" + " "*20 + "║")
            self.get_logger().info("╚" + "═"*58 + "╝")
        except Exception as e:
            self.get_logger().warn(f"⚠️  Impossible de récupérer les infos réseau: {e}")
    
    async def handle_websocket(self, websocket, path=None):
        """Gère une connexion WebSocket avec logging détaillé"""
        client_ip = websocket.remote_address[0]
        client_port = websocket.remote_address[1]
        client_id = f"{client_ip}:{client_port}"
        connection_time = datetime.now()
        
        path_str = str(path) if path is not None else "/"

        self.connected_clients.add(websocket)
        
        # Log détaillé de connexion
        self.get_logger().info("╔" + "═"*58 + "╗")
        self.get_logger().info("║" + " "*16 + "🔌 NOUVELLE CONNEXION" + " "*21 + "║")
        self.get_logger().info("╠" + "═"*58 + "╣")
        self.get_logger().info(f"║ 🌐 IP: {client_ip:<48} ║")
        self.get_logger().info(f"║ 🔢 Port: {client_port:<46} ║")
        self.get_logger().info(f"║ 🆔 ID: {client_id:<49} ║")
        self.get_logger().info(f"║ 🕐 Heure: {connection_time.strftime('%H:%M:%S.%f')[:-3]:<43} ║")
        self.get_logger().info(f"║ 👥 Total clients: {len(self.connected_clients):<37} ║")
        self.get_logger().info(f"║ 📂 Path: {path_str:<47} ║")
        self.get_logger().info("╚" + "═"*58 + "╝")
        
        try:
            # Message d'accueil
            welcome = {
                'type': 'connected',
                'robot': self.robot_namespace,
                'server_time': connection_time.isoformat(),
                'config': {
                    'max_linear_vel': self.max_linear_vel,
                    'min_linear_vel': self.min_linear_vel,
                    'max_angular_vel': self.max_angular_vel
                }
            }
            
            welcome_json = json.dumps(welcome, indent=2)
            
            self.get_logger().info(f"📤 [WS→{client_id}] Envoi message d'accueil")
            self.get_logger().info(f"📦 Payload ({len(welcome_json)} octets):")
            for line in welcome_json.split('\n'):
                self.get_logger().info(f"   {line}")
            
            await websocket.send(welcome_json)
            self.get_logger().info(f"✅ [WS→{client_id}] Message d'accueil envoyé")
            
            # Boucle de réception
            msg_count = 0
            self.get_logger().info(f"👂 [WS←{client_id}] En attente de messages...")
            
            async for message in websocket:
                msg_count += 1
                receive_time = datetime.now()
                
                # Log de réception détaillé
                self.get_logger().info("┌" + "─"*58 + "┐")
                self.get_logger().info(f"│ 📥 MESSAGE #{msg_count:<46} │")
                self.get_logger().info("├" + "─"*58 + "┤")
                self.get_logger().info(f"│ 🆔 Client: {client_id:<45} │")
                self.get_logger().info(f"│ 🕐 Heure: {receive_time.strftime('%H:%M:%S.%f')[:-3]:<46} │")
                self.get_logger().info(f"│ 📏 Taille: {len(message)} octets{' '*(44-len(str(len(message))))} │")
                self.get_logger().info("└" + "─"*58 + "┘")
                
                # Log du contenu brut
                self.get_logger().info(f"📄 Contenu brut:")
                if len(message) <= 500:
                    self.get_logger().info(f"   {message}")
                else:
                    self.get_logger().info(f"   {message[:500]}... (tronqué)")
                
                try:
                    data = json.loads(message)
                    msg_type = data.get('type', 'unknown')
                    
                    self.get_logger().info(f"📦 Message parsé:")
                    self.get_logger().info(f"   ├─ Type: {msg_type}")
                    
                    # Log du JSON complet
                    json_str = json.dumps(data, indent=2)
                    self.get_logger().info(f"   └─ Données:")
                    for line in json_str.split('\n'):
                        self.get_logger().info(f"      {line}")
                    
                    # Traitement selon le type
                    if msg_type == 'cmd_vel':
                        await self._process_cmd_vel(websocket, data, client_id, msg_count)
                    
                    elif msg_type == 'emergency_stop':
                        await self._process_emergency_stop(websocket, client_id, msg_count)
                    
                    elif msg_type == 'get_status':
                        await self._process_get_status(websocket, client_id, msg_count)
                    
                    elif msg_type == 'ping':
                        await self._process_ping(websocket, client_id, msg_count)
                    
                    else:
                        self.get_logger().warn(f"❓ [WS←{client_id}] Type inconnu: {msg_type}")
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'reason': f'Unknown message type: {msg_type}',
                            'received_type': msg_type
                        }))
                
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"⚠️  [WS←{client_id}] ERREUR JSON:")
                    self.get_logger().error(f"   ├─ Position: {e.pos}")
                    self.get_logger().error(f"   ├─ Ligne: {e.lineno}, Colonne: {e.colno}")
                    self.get_logger().error(f"   ├─ Message: {e.msg}")
                    self.get_logger().error(f"   └─ Contenu: {message[:200]}")
                    
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'reason': 'Invalid JSON',
                        'details': str(e)
                    }))
                
                except Exception as e:
                    self.get_logger().error(f"💥 [WS←{client_id}] ERREUR TRAITEMENT:")
                    self.get_logger().error(f"   ├─ Type: {type(e).__name__}")
                    self.get_logger().error(f"   ├─ Message: {e}")
                    self.get_logger().error(f"   └─ Traceback:")
                    for line in traceback.format_exc().split('\n'):
                        if line.strip():
                            self.get_logger().error(f"      {line}")
        
        except websockets.exceptions.ConnectionClosed as e:
            duration = (datetime.now() - connection_time).total_seconds()
            
            self.get_logger().info("╔" + "═"*58 + "╗")
            self.get_logger().info("║" + " "*18 + "🔌 DÉCONNEXION" + " "*25 + "║")
            self.get_logger().info("╠" + "═"*58 + "╣")
            self.get_logger().info(f"║ 🆔 Client: {client_id:<44} ║")
            self.get_logger().info(f"║ 🔢 Code: {e.code:<47} ║")
            self.get_logger().info(f"║ 📝 Raison: {str(e.reason)[:43]:<44} ║")
            self.get_logger().info(f"║ 📊 Messages traités: {msg_count:<34} ║")
            self.get_logger().info(f"║ ⏱️  Durée: {duration:.2f}s{' '*(43-len(f'{duration:.2f}'))} ║")
            self.get_logger().info("╚" + "═"*58 + "╝")
        
        except Exception as e:
            self.get_logger().error(f"💥 [WS] ERREUR CONNEXION ({client_id}):")
            self.get_logger().error(f"   ├─ {type(e).__name__}: {e}")
            self.get_logger().error(f"   └─ Traceback:")
            for line in traceback.format_exc().split('\n'):
                if line.strip():
                    self.get_logger().error(f"      {line}")
        
        finally:
            if websocket in self.connected_clients:
                self.connected_clients.remove(websocket)
            
            self.get_logger().info(f"👋 [WS] Fin connexion {client_id}")
            self.get_logger().info(f"   └─ Clients restants: {len(self.connected_clients)}")
    
    async def _process_cmd_vel(self, websocket, data, client_id, msg_num):
        """Traite une commande cmd_vel"""
        self.commands_received += 1
        process_start = datetime.now()
        
        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        
        self.get_logger().info(f"🎮 [CMD_VEL #{msg_num}] Commande reçue:")
        self.get_logger().info(f"   ├─ Source: {client_id}")
        self.get_logger().info(f"   ├─ linear_x:  {linear_x:>8.6f} m/s")
        self.get_logger().info(f"   └─ angular_z: {angular_z:>8.6f} rad/s")
        
        # Validation simple (clamp uniquement)
        is_valid, safe_linear, safe_angular, reason = \
            self.velocity_limiter.validate_command(linear_x, angular_z)
        
        self.commands_accepted += 1
        
        # Publication sur ROS
        twist = Twist()
        twist.linear.x = safe_linear
        twist.angular.z = safe_angular
        
        self.get_logger().info(f"✅ [CMD_VEL #{msg_num}] Commande PUBLIÉE:")
        self.get_logger().info(f"   ├─ Raison: {reason}")
        self.get_logger().info(f"   ├─ Publié linear:  {safe_linear:>8.6f} m/s")
        self.get_logger().info(f"   ├─ Publié angular: {safe_angular:>8.6f} rad/s")
        self.get_logger().info(f"   └─ Topic: /{self.robot_namespace}/cmd_vel")
        
        self.cmd_vel_pub.publish(twist)
        self.last_status = reason
        
        # Stats
        accept_rate = (self.commands_accepted / self.commands_received * 100) if self.commands_received > 0 else 0
        self.get_logger().info(f"📊 [STATS] Reçues: {self.commands_received} | "
                             f"Acceptées: {self.commands_accepted} ({accept_rate:.1f}%)")
        
        # Confirmation au client
        response = {
            'type': 'cmd_accepted',
            'linear_x': safe_linear,
            'angular_z': safe_angular,
            'reason': reason,
            'msg_num': msg_num,
            'timestamp': datetime.now().isoformat()
        }
        
        await websocket.send(json.dumps(response))
        
        process_time = (datetime.now() - process_start).total_seconds() * 1000
        self.get_logger().info(f"⏱️  [CMD_VEL #{msg_num}] Traité en {process_time:.2f}ms")
    
    async def _process_emergency_stop(self, websocket, client_id, msg_num):
        """Traite un arrêt d'urgence"""
        self.get_logger().warn("╔" + "═"*58 + "╗")
        self.get_logger().warn("║" + " "*16 + "🚨 ARRÊT D'URGENCE" + " "*23 + "║")
        self.get_logger().warn("╠" + "═"*58 + "╣")
        self.get_logger().warn(f"║ 🆔 Demandé par: {client_id:<40} ║")
        self.get_logger().warn(f"║ 📨 Message #{msg_num:<44} ║")
        self.get_logger().warn(f"║ 🕐 Heure: {datetime.now().strftime('%H:%M:%S.%f')[:-3]:<46} ║")
        self.get_logger().warn("╚" + "═"*58 + "╝")
        
        # Arrêt immédiat
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().warn(f"🛑 [EMERGENCY] Robot ARRÊTÉ sur /{self.robot_namespace}/cmd_vel")
        
        # Confirmation
        response = {
            'type': 'emergency_stop_ack',
            'timestamp': datetime.now().isoformat(),
            'msg_num': msg_num
        }
        
        await websocket.send(json.dumps(response))
        self.get_logger().info(f"✅ [EMERGENCY] Confirmation envoyée à {client_id}")
    
    async def _process_get_status(self, websocket, client_id, msg_num):
        """Traite une demande de status"""
        self.get_logger().info(f"📊 [STATUS] Demande #{msg_num} de {client_id}")
        
        uptime = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        status = {
            'type': 'status',
            'timestamp': datetime.now().isoformat(),
            'uptime_seconds': uptime,
            'commands_received': self.commands_received,
            'commands_accepted': self.commands_accepted,
            'last_status': self.last_status,
            'connected_clients': len(self.connected_clients)
        }
        
        status_json = json.dumps(status, indent=2)
        self.get_logger().info(f"📤 [STATUS] Envoi:")
        for line in status_json.split('\n')[:10]:
            self.get_logger().info(f"   {line}")
        
        await websocket.send(status_json)
        self.get_logger().info(f"✅ [STATUS] Status envoyé à {client_id}")
    
    async def _process_ping(self, websocket, client_id, msg_num):
        """Traite un ping"""
        self.get_logger().info(f"🏓 [PING] Reçu de {client_id}")
        
        response = {
            'type': 'pong',
            'timestamp': datetime.now().isoformat(),
            'msg_num': msg_num
        }
        
        await websocket.send(json.dumps(response))
        self.get_logger().info(f"✅ [PONG] Réponse envoyée à {client_id}")
    
    async def broadcast_status(self):
        """Broadcast périodique du status"""
        broadcast_count = 0
        
        self.get_logger().info("📡 [BROADCAST] Service de broadcast démarré")
        
        while self.running:
            await asyncio.sleep(5.0)
            
            if self.connected_clients:
                broadcast_count += 1
                uptime = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                
                status_msg = {
                    'type': 'status_broadcast',
                    'timestamp': datetime.now().isoformat(),
                    'broadcast_num': broadcast_count,
                    'uptime_seconds': uptime,
                    'last_status': self.last_status,
                    'commands_stats': {
                        'received': self.commands_received,
                        'accepted': self.commands_accepted
                    },
                    'connected_clients': len(self.connected_clients)
                }
                
                if broadcast_count % 12 == 0:
                    self.get_logger().info(f"📡 [BROADCAST #{broadcast_count}] Status envoyé à {len(self.connected_clients)} client(s)")
                    self.get_logger().info(f"   ├─ Uptime: {uptime:.1f}s")
                    self.get_logger().info(f"   ├─ Commandes: {self.commands_received} reçues, {self.commands_accepted} acceptées")
                    self.get_logger().info(f"   └─ Status: {self.last_status}")
                
                try:
                    websockets.broadcast(self.connected_clients, json.dumps(status_msg))
                except Exception as e:
                    self.get_logger().error(f"⚠️  [BROADCAST] Erreur: {e}")
    
    def run_websocket_server(self):
        """Lance le serveur WebSocket"""
        self.get_logger().info("╔" + "═"*58 + "╗")
        self.get_logger().info("║" + " "*12 + "🚀 DÉMARRAGE SERVEUR WEBSOCKET" + " "*16 + "║")
        self.get_logger().info("╚" + "═"*58 + "╝")
        
        async def main():
            try:
                self.get_logger().info(f"🔧 [WS-SERVER] Configuration:")
                self.get_logger().info(f"   ├─ Host: {self.ws_host}")
                self.get_logger().info(f"   ├─ Port: {self.ws_port}")
                self.get_logger().info(f"   └─ Namespace: {self.robot_namespace}")
                
                self.get_logger().info(f"🎬 [WS-SERVER] Lancement du serveur...")
                
                async with websockets.serve(
                    self.handle_websocket,
                    self.ws_host,
                    self.ws_port,
                    ping_interval=20,
                    ping_timeout=10,
                    compression=None
                ):
                    self.get_logger().info("╔" + "═"*58 + "╗")
                    self.get_logger().info("║" + " "*12 + "✅ SERVEUR WEBSOCKET ACTIF" + " "*19 + "║")
                    self.get_logger().info("╠" + "═"*58 + "╣")
                    
                    try:
                        hostname = socket.gethostname()
                        local_ip = socket.gethostbyname(hostname)
                        
                        self.get_logger().info(f"║ 🌐 URLs de connexion:" + " "*35 + "║")
                        self.get_logger().info(f"║    • Local:   ws://localhost:{self.ws_port}{' '*(27-len(str(self.ws_port)))} ║")
                        self.get_logger().info(f"║    • LAN:     ws://{local_ip}:{self.ws_port}{' '*(27-len(local_ip)-len(str(self.ws_port)))} ║")
                    except:
                        self.get_logger().info(f"║ 🌐 URL: ws://{self.ws_host}:{self.ws_port}{' '*(36-len(self.ws_host)-len(str(self.ws_port)))} ║")
                    
                    self.get_logger().info("╠" + "═"*58 + "╣")
                    self.get_logger().info(f"║ 🎯 En attente de connexions..." + " "*26 + "║")
                    self.get_logger().info(f"║ 🤖 Robot: {self.robot_namespace:<44} ║")
                    self.get_logger().info(f"║ 📊 Prêt à recevoir des commandes cmd_vel" + " "*15 + "║")
                    self.get_logger().info("╚" + "═"*58 + "╝")
                    
                    # Broadcast task
                    broadcast_task = asyncio.create_task(self.broadcast_status())
                    
                    self.get_logger().info("✅ [WS-SERVER] Serveur opérationnel!")
                    self.get_logger().info("")
                    self.get_logger().info("📋 COMMANDES DISPONIBLES:")
                    self.get_logger().info("   • cmd_vel        - Envoyer une commande de vitesse")
                    self.get_logger().info("   • emergency_stop - Arrêt d'urgence")
                    self.get_logger().info("   • get_status     - Obtenir le statut")
                    self.get_logger().info("   • ping           - Test de connexion")
                    self.get_logger().info("")
                    
                    await asyncio.Future()
            
            except OSError as e:
                self.get_logger().error("╔" + "═"*58 + "╗")
                self.get_logger().error("║" + " "*14 + "❌ ERREUR DÉMARRAGE SERVEUR" + " "*17 + "║")
                self.get_logger().error("╠" + "═"*58 + "╣")
                
                if e.errno == 98 or e.errno == 48:
                    self.get_logger().error(f"║ ⚠️  Port {self.ws_port} déjà utilisé!" + " "*(36-len(str(self.ws_port))) + "║")
                    self.get_logger().error("║" + " "*58 + "║")
                    self.get_logger().error("║ 💡 Solutions:" + " "*44 + "║")
                    self.get_logger().error(f"║    lsof -i :{self.ws_port}{' '*(44-len(str(self.ws_port)))} ║")
                else:
                    self.get_logger().error(f"║ Erreur: {str(e)[:50]:<50} ║")
                
                self.get_logger().error("╚" + "═"*58 + "╝")
            
            except Exception as e:
                self.get_logger().error("╔" + "═"*58 + "╗")
                self.get_logger().error("║" + " "*18 + "💥 ERREUR SERVEUR" + " "*23 + "║")
                self.get_logger().error("╠" + "═"*58 + "╣")
                self.get_logger().error(f"║ Type: {type(e).__name__:<49} ║")
                self.get_logger().error(f"║ Message: {str(e)[:48]:<48} ║")
                self.get_logger().error("╚" + "═"*58 + "╝")
        
        asyncio.run(main())
    
    def shutdown(self):
        """Arrêt propre du bridge"""
        self.get_logger().info("╔" + "═"*58 + "╗")
        self.get_logger().info("║" + " "*20 + "🛑 ARRÊT DU BRIDGE" + " "*21 + "║")
        self.get_logger().info("╚" + "═"*58 + "╝")
        
        self.running = False
        
        # Arrêt du robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("   ├─ ✅ Robot arrêté")
        
        # Stats finales
        uptime = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.get_logger().info("╔" + "═"*58 + "╗")
        self.get_logger().info("║" + " "*20 + "📊 STATISTIQUES FINALES" + " "*15 + "║")
        self.get_logger().info("╠" + "═"*58 + "╣")
        self.get_logger().info(f"║ ⏱️  Uptime: {uptime:.2f}s{' '*(47-len(f'{uptime:.2f}'))} ║")
        self.get_logger().info(f"║ 📥 Commandes reçues: {self.commands_received:<33} ║")
        self.get_logger().info(f"║ ✅ Commandes acceptées: {self.commands_accepted:<30} ║")
        
        if self.commands_received > 0:
            accept_rate = (self.commands_accepted / self.commands_received * 100)
            self.get_logger().info(f"║ 📊 Taux d'acceptation: {accept_rate:.1f}%{' '*(30-len(f'{accept_rate:.1f}'))} ║")
        
        self.get_logger().info("╚" + "═"*58 + "╝")
        
        self.destroy_node()
        self.get_logger().info("✅ [SHUTDOWN] Bridge arrêté proprement")


def main(args=None):
    """Point d'entrée principal"""
    print("\n" + "╔" + "═"*58 + "╗")
    print("║" + " "*8 + "🚀 TURTLEBOT WEBSOCKET BRIDGE ROS2 🚀" + " "*13 + "║")
    print("║" + " "*12 + "Version 2.1 - Bridge Simple & Propre" + " "*9 + "║")
    print("╚" + "═"*58 + "╝\n")
    
    rclpy.init(args=args)
    
    bridge = None
    
    try:
        bridge = ROSBridge()
        
        print("🔧 Démarrage du serveur WebSocket...\n")
        ws_thread = Thread(target=bridge.run_websocket_server, daemon=True)
        ws_thread.start()
        
        print("✅ Bridge initialisé")
        print("⏳ Attendez le message '✅ SERVEUR WEBSOCKET ACTIF'...\n")
        
        import time
        time.sleep(2)
        
        print("🔄 Démarrage du spin ROS2...\n")
        rclpy.spin(bridge)
    
    except KeyboardInterrupt:
        print("\n╔" + "═"*58 + "╗")
        print("║" + " "*12 + "🛑 ARRÊT DEMANDÉ PAR L'UTILISATEUR" + " "*13 + "║")
        print("╚" + "═"*58 + "╝")
    
    except Exception as e:
        print("\n╔" + "═"*58 + "╗")
        print("║" + " "*20 + "💥 ERREUR FATALE" + " "*23 + "║")
        print("╠" + "═"*58 + "╣")
        print(f"║ {type(e).__name__}: {str(e)[:45]:<45} ║")
        print("╚" + "═"*58 + "╝\n")
        traceback.print_exc()
    
    finally:
        if bridge is not None:
            try:
                bridge.shutdown()
            except:
                pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("\n✅ Programme terminé\n")


if __name__ == '__main__':
    main()