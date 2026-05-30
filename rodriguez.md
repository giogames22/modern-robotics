### 1. Fórmula de Rodrigues para Rotaciones en $SO(3)$

Dada una velocidad angular unitaria $\omega \in \mathbb{R}^3$ representada como una matriz antisimétrica $[\omega] \in so(3)$, y un ángulo de rotación $\theta$, la matriz de rotación final $R \in SO(3)$ (el mapa exponencial) se calcula mediante la Fórmula de Rodrigues:

$$
e^{[\omega]\theta} = I + \sin(\theta)[\omega] + (1 - \cos(\theta))[\omega]^2
$$

Donde $I$ es la matriz identidad de $3 \times 3$.

**Implementación manual en Python:**

```python
# FORMULA DE RODRIGUEZ
import numpy as np

print("Introduce las componentes del vector que servirá como EJE de rotación.")
#VALORES DEL VECTOR QUE SERA EL EJE DE ROTACION
kx = float(input("valor del eje x:"))
ky = float(input("valor del eje y:"))
kz = float(input("valor del eje z:"))

#VECTOR DE ROTACION K
eje_k = np.array([kx, ky, kz], dtype=float)

#MAGNITUD DE K
magnitud = np.linalg.norm(eje_k)

#REVISION DEL VECTOR K
if magnitud == 0:
    print("\n¡Error! El eje de rotación no puede ser el vector nulo (0, 0, 0).")
    print("Asignando un eje por defecto (Eje Z: 0, 0, 1) para continuar.")
    eje_k = np.array([0.0, 0.0, 1.0])

#CALCULO DE VECTOR UNITARIO
else:
    eje_k = eje_k/magnitud
    print("-----------------vector unitario-------------------")
    print(eje_k)
    

#VECTOR ORIGINAL A DESPLAZAR 
print("Introduce las componentes del vector que deseas rotar.")
vx = float(input("valor del componente x:"))
vy = float(input("valor del componente y:"))
vz = float(input("valor del componente z:"))

#VECTOR A ROTAR V
vector_v = np.array([vx,vy,vz], dtype=float)

#CREACION DE ANTISIMETRICAS
kx_u = eje_k[0]
ky_u = eje_k[1]
kz_u = eje_k[2]

matriz_k_cruz = np.array([
    [  0.0, -kz_u,  ky_u],
    [ kz_u,   0.0, -kx_u],
    [-ky_u,  kx_u,   0.0]
], dtype=float)

#ANGULO A ROTAR
angulo_grados = float(input("Ángulo en grados (°): "))
theta = np.radians(angulo_grados)

#CREACION DE LA MATRIZ IDENTIDAD
matriz_I = np.eye(3)

#ELEVAR LA MATRIZ ANTISIMETRICA AL CUADRADO
matriz_k_cuadrado = matriz_k_cruz @ matriz_k_cruz

#MATRIZ DE RODRIGUEZ 
Rot = matriz_I + (np.sin(theta) * matriz_k_cruz) + ((1 - np.cos(theta)) * matriz_k_cuadrado)
print("\nMatriz de Rotación Final:")
print(np.round(Rot, 4))
```

---

## Implementación Práctica en Python

A continuación, presento dos formas de implementar estos conceptos en código. Primero, utilizando herramientas profesionales de la industria para resolver la matemática abstracta y, segundo, una simulación visual completa construida desde cero.

### 1. Forma Reducida: Uso de la librería `modern_robotics` (Rotación Pura)

En la práctica profesional, no calculamos las exponenciales de matrices a mano. Utilizamos librerías especializadas como `modern_robotics` (basada en el libro homónimo). El siguiente código demuestra cómo obtener la matriz de rotación final en $SO(3)$ a partir de un eje arbitrario y un ángulo, reduciendo toda la matemática a un par de líneas funcionales:

```python
import modern_robotics as mr
import numpy as np

# 1. Datos iniciales (Eje K original, Vector V a rotar, y Ángulo)
k = np.array([10, 5, 4]) 
v = np.array([1, 0, 0])
theta = np.radians(90)

# 2. Corrección: Normalizar K para que la librería funcione bien
k_unitario = k / np.linalg.norm(k)

# 3. Magia de Modern Robotics (Antisimétrica y Rodrigues)
Rot = mr.MatrixExp3(mr.VecToso3(k_unitario) * theta)

# 4. Mostrar solo la matriz de salida
print("Matriz de Rotación (R):\n", np.round(Rot, 4))
```

---


**En resumen (La intuición robótica):**

Pasar de $SO(3)$ a $SE(3)$ significa que dejamos de modelar un "eje girando en el vacío" y empezamos a modelar **un brazo físico real**, con motores desplazados de la base (offsets) y eslabones que barren un volumen en el espacio físico al moverse. Por eso, en $SE(3)$ la velocidad angular ($\omega$) se acompaña de una velocidad lineal inducida ($v$), formando lo que llamamos un **Twist**.




### 2. Forma Visual: Simulación 3D en $SE(3)$ desde cero

Para entender realmente lo que sucede bajo el capó, el siguiente script implementa la matemática de los Grupos de Lie y la Fórmula de Rodrigues desde cero. 

Además, da el salto de $SO(3)$ a $SE(3)$ (Cuerpo Rígido): incorpora un punto de apoyo (offset del motor), calcula la traslación inducida (Twist) y grafica en 3D el eslabón original, la trayectoria del movimiento y la posición final utilizando `matplotlib`.

```python
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. ENTRADA DE DATOS
# ==========================================
print("=== CONFIGURACIÓN DEL EJE DE ROTACIÓN ===")
kx = float(input("Componente X del eje: "))
ky = float(input("Componente Y del eje: "))
kz = float(input("Componente Z del eje: "))
eje_k = np.array([kx, ky, kz], dtype=float)

magnitud = np.linalg.norm(eje_k)
if magnitud == 0:
    print("¡Error! Eje nulo. Asignando Eje Z por defecto (0,0,1).")
    eje_k = np.array([0.0, 0.0, 1.0])
else:
    eje_k = eje_k / magnitud

print("\n=== CONFIGURACIÓN DEL PUNTO DE APOYO (MOTOR) ===")
qx = float(input("Coordenada X del apoyo: "))
qy = float(input("Coordenada Y del apoyo: "))
qz = float(input("Coordenada Z del apoyo: "))
apoyo = np.array([qx, qy, qz], dtype=float)

print("\n=== CONFIGURACIÓN DEL PUNTO A MOVER (PUNTA DEL ESLABÓN) ===")
vx = float(input("Coordenada X del punto inicial: "))
vy = float(input("Coordenada Y del punto inicial: "))
vz = float(input("Coordenada Z del punto inicial: "))
vector_v = np.array([vx, vy, vz], dtype=float)

angulo_grados = float(input("\nÁngulo a rotar en grados (°): "))
theta = np.radians(angulo_grados)

# ==========================================
# 2. CÁLCULOS MATEMÁTICOS SE(3) Y TRAYECTORIA
# ==========================================
v_twist = np.cross(-eje_k, apoyo)
matriz_k_cruz = np.array([
    [       0.0, -eje_k[2],  eje_k[1]],
    [ eje_k[2],        0.0, -eje_k[0]],
    [-eje_k[1],  eje_k[0],       0.0]
], dtype=float)

matriz_I = np.eye(3)
matriz_k_cuadrado = matriz_k_cruz @ matriz_k_cruz

vector_homogeneo = np.array([vector_v[0], vector_v[1], vector_v[2], 1.0])

num_puntos = 50
angulos_intermedios = np.linspace(0, theta, num_puntos)

trayectoria_x = []
trayectoria_y = []
trayectoria_z = []

for ang in angulos_intermedios:
    # Rodrigues para la rotación pura
    Rot_i = matriz_I + (np.sin(ang) * matriz_k_cruz) + ((1 - np.cos(ang)) * matriz_k_cuadrado)
    
    # Cálculo de la traslación en el espacio
    G_i = (matriz_I * ang) + ((1 - np.cos(ang)) * matriz_k_cruz) + ((ang - np.sin(ang)) * matriz_k_cuadrado)
    p_i = G_i @ v_twist
    
    # Construcción de la matriz de Transformación Homogénea 4x4
    T_i = np.eye(4)
    T_i[0:3, 0:3] = Rot_i
    T_i[0:3, 3] = p_i
    
    # Calcular la posición intermedia
    pos_intermedia = T_i @ vector_homogeneo
    trayectoria_x.append(pos_intermedia[0])
    trayectoria_y.append(pos_intermedia[1])
    trayectoria_z.append(pos_intermedia[2])

punto_final_3d = np.array([trayectoria_x[-1], trayectoria_y[-1], trayectoria_z[-1]])

# ==========================================
# 3. SALIDA DE RESULTADOS
# ==========================================
print("\n" + "="*50)
print(f"Punto Original: {vector_v}")
print(f"NUEVA POSICIÓN: {np.round(punto_final_3d, 4)}")
print("="*50)

# ==========================================
# 4. VISUALIZACIÓN 3D CON MATPLOTLIB
# ==========================================
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Graficar los puntos principales
ax.scatter(*vector_v, color='blue', s=60, label='Punta Inicial')
ax.scatter(*punto_final_3d, color='red', s=60, label='Punta Final')
ax.scatter(*apoyo, color='green', s=100, marker='^', label='Motor (Apoyo)')

# Graficar el Eje de Rotación
t = np.linspace(-10, 10, 100)
linea_x = apoyo[0] + t * eje_k[0]
linea_y = apoyo[1] + t * eje_k[1]
linea_z = apoyo[2] + t * eje_k[2]
ax.plot(linea_x, linea_y, linea_z, color='green', linestyle='--', alpha=0.5, label='Eje de rotación')

# Graficar la trayectoria
ax.plot(trayectoria_x, trayectoria_y, trayectoria_z, color='orange', linewidth=2, label='Trayectoria')

# Graficar el Eslabón (Brazo Físico)
ax.plot([apoyo[0], vector_v[0]], 
        [apoyo[1], vector_v[1]], 
        [apoyo[2], vector_v[2]], 
        color='darkblue', linewidth=4, label='Eslabón Inicial')

ax.plot([apoyo[0], punto_final_3d[0]], 
        [apoyo[1], punto_final_3d[1]], 
        [apoyo[2], punto_final_3d[2]], 
        color='darkred', linewidth=4, label='Eslabón Final')

# Configuración visual de la gráfica
ax.set_xlabel('Eje X')
ax.set_ylabel('Eje Y')
ax.set_zlabel('Eje Z')
ax.set_title('Movimiento del Eslabón Robótico en el Espacio')
ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5))

# Ajustar los límites de la gráfica
todos_los_valores = list(vector_v) + list(punto_final_3d) + list(apoyo) + trayectoria_x + trayectoria_y + trayectoria_z
max_val = np.max(np.abs(todos_los_valores)) + 2
ax.set_xlim([-max_val, max_val])
ax.set_ylim([-max_val, max_val])
ax.set_zlim([-max_val, max_val])

plt.tight_layout()
plt.show()
```
---
### 3. Forma Reducida: Uso de `modern_robotics` (Movimiento Completo en $SE(3)$)

Al igual que con las rotaciones puras, el cálculo del movimiento en el espacio completo (rotación + traslación) se simplifica inmensamente usando la librería `modern_robotics`. 

En lugar de calcular la matriz de rotación por un lado y la traslación espacial por el otro, agrupamos el eje de rotación ($\omega$) y la velocidad lineal inducida ($v = -\omega \times q$) en un solo vector de 6 dimensiones llamado **Twist espacia**l ($S$). Luego, aplicamos la exponencial de matriz para $SE(3)$ (`MatrixExp6`).

```python
import modern_robotics as mr
import numpy as np

# 1. Datos iniciales (Motor, Eje, Punta y Ángulo)
k = np.array([0, 0, 1])                # Eje Z
apoyo = np.array([2, 0, 0])            # Coordenada 'q' (Posición del motor)
vector_v = np.array([5, 0, 0])         # Posición original de la punta
theta = np.radians(90)                 # Ángulo a rotar

# Normalizar el eje de rotación (Omega)
omega = k / np.linalg.norm(k)

# 2. Construir el Eje de Tornillo / Twist (S)
# La velocidad lineal (v) se calcula como el producto cruz de -omega por el punto de apoyo
v_lineal = np.cross(-omega, apoyo)
S = np.concatenate((omega, v_lineal))  # Twist S de 6 elementos [w_x, w_y, w_z, v_x, v_y, v_z]

# 3. Magia de Modern Robotics para SE(3)
# VecTose3 convierte el Twist(6) en matriz homogénea(4x4) de álgebra de Lie se(3)
# MatrixExp6 aplica la Fórmula de Rodrigues generalizada
T_final = mr.MatrixExp6(mr.VecTose3(S) * theta)

# 4. Calcular la nueva posición física de la punta
# Convertimos la punta a coordenadas homogéneas agregando un 1.0 al final
vector_homogeneo = np.array([vector_v[0], vector_v[1], vector_v[2], 1.0])
pos_final = T_final @ vector_homogeneo

print("Matriz de Transformación (T_final):\n", np.round(T_final, 4))
print("\nPosición Final de la Punta (X, Y, Z):")
print(np.round(pos_final[0:3], 4))
```
