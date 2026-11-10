"""Ręczne sterowanie samochodem strzałkami / WASD → /cmd_vel."""

import sys
import select
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP = 'Sterowanie: strzałki lub wasd (małe litery) | spacja = stop | q = wyjście'

# ŹLE DZIAŁA STRZAŁKAMI WIĘC JEŹDZIĆ WSADEM

class Manual(Node):
    def __init__(self):
        super().__init__('manual')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Publisher wysyłający prędkość liniową i kątową robota na topic /cmd_vel; 10 oznacza głębokość kolejki wiadomości
        self.speed = 1.0  # Zadana prędkość liniowa robota [m/s]
        self.turn = 0.6  # Zadana prędkość kątowa robota [rad/s]
        self.cmd = Twist()  # Wiadomość przechowująca aktualną prędkość liniową (linear) i kątową (angular)
        self.create_timer(0.05, self._on_timer)  # Timer wywołujący funkcję _on_timer co 0.05 s (20 razy na sekundę)
        self.get_logger().info(HELP)  # Wypisanie komunikatu HELP w terminalu za pomocą loggera ROS 2

    def _on_timer(self):
        self.pub.publish(self.cmd) # Wysłanie aktualnej komendy prędkości robota na topic /cmd_vel

    def handle_key(self, key: str) -> bool:
        # Tylko małe wasd — wielkie A/D kolidują z końcówkami strzałek.
        if key in ('w'):
            self.cmd.linear.x = self.speed
            self.cmd.angular.z = 0.0
        elif key in ('s'):
            self.cmd.linear.x = -self.speed
            self.cmd.angular.z = 0.0
        elif key in ('d'):  # prawo
            self.cmd.angular.z = -self.turn  
        elif key in ('a'):  # lewo
            self.cmd.angular.z = self.turn   
        elif key == ' ':    # spacja - zatrzymanie
            self.cmd = Twist()
        elif key in ('q', '\x03'):
            return False
        else:
            return True

        self.get_logger().info(
            f'→ linear.x={self.cmd.linear.x:.1f} angular.z={self.cmd.angular.z:.1f}'
        )
        self.pub.publish(self.cmd)  # publikacja komendy
        return True


def _stdin_ready(timeout: float) -> bool:
    # [sys.stdin] – sprawdza, czy można odczytać dane z klawiatury.
    return bool(select.select([sys.stdin], [], [], timeout)[0])

# funkcja odczytuje jeden klawisz z klawiatury i go zwraca
# nie blokuje programu etc.
# bierze pod uwagę klawisze specjalne które inaczej się kodują
def get_key(timeout: float = 0.1) -> str:
    if not _stdin_ready(timeout):
        return ''
    key = sys.stdin.read(1)
    if key != '\x1b':
        return key

    if not _stdin_ready(0.05):
        return ''
    key += sys.stdin.read(1)
    if key[1] in ('[', 'O'):
        if not _stdin_ready(0.05):
            return ''
        key += sys.stdin.read(1)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)  # Zapisanie aktualnych ustawień terminala, aby można je było później przywrócić
    rclpy.init(args=args)  # Inicjalizacja biblioteki ROS 2
    node = Manual()  # Utworzenie węzła ROS 2 odpowiedzialnego za ręczne sterowanie robotem

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)  # Utworzenie wątku obsługującego komunikację ROS 2 i callbacki węzła
    spin_thread.start()  # Uruchomienie wątku ROS 2

    tty.setcbreak(sys.stdin.fileno())  # Ustawienie terminala w tryb cbreak, umożliwiający odczyt klawiszy bez naciskania Enter
    try:
        while rclpy.ok():  # Wykonywanie pętli, dopóki ROS 2 działa
            key = get_key(0.1)  # Odczytanie klawisza z maksymalnym czasem oczekiwania 0.1 s
            if key and not node.handle_key(key):  # Przekazanie naciśniętego klawisza do obsługi; zakończenie pętli, jeśli handle_key zwróci False
                break
    except KeyboardInterrupt:
        pass  # Zignorowanie przerwania programu przez Ctrl+C i przejście do sprzątania

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Przywrócenie pierwotnych ustawień terminala
        node.cmd = Twist()  # Wyzerowanie wszystkich składowych prędkości liniowej i kątowej
        node.pub.publish(Twist())  # Wysłanie zerowej prędkości do robota, aby go zatrzymać
        node.destroy_node()  # Usunięcie węzła ROS 2 i zwolnienie jego zasobów
        rclpy.shutdown()  # Zakończenie działania ROS 2


if __name__ == '__main__':
    main()
