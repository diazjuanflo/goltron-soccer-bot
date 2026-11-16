# -*- coding: utf-8 -*-
"""
================================================================================
PROYECTO ROBOGOL SD2026-2 (USCO) - SCRIPT CAD/DFM/DFA COMPLETO (CORREGIDO V4)
Archivo  : robogol_cad_v2_completo_100.py
Unidades : Milimetros (1 unidad Blender = 1.0 mm | scale_length = 0.001)
================================================================================
Ejecutar desde la pestana Scripting de Blender, en modo Objeto.
Incluye:
  1. Parametros maestros centralizados y holguras DFM/DFA diferenciadas.
  2. Esquinas traseras y delanteras 100% cerradas con chaflanes internos en el
     vaciado del chasis, en el labio de la tapa y columnas esquineras remetidas.
  3. Correccion integral de todas las intersecciones fisicas y mecanicas:
     - Aberturas superiores y rebajes de labio en Tapa P02 para paso y giro libre
       de las llantas de Ø65 mm.
     - Cajeras de rueda abiertas hasta Z > 60 mm en Chasis P01 y desplazamiento
       lateral de ruedas (X = +-64.5 mm) y motores TT (X = +-32.5 mm).
     - Elevacion de la Placa Portadora PCB (ALTO_POSTES_PCB = 27.2 mm) y
       reubicacion de postes al pasillo central (X = +-14.5 mm) fuera de motores.
     - Abrazaderas P03/P04 con paso de eje, pilares continuos y tornillos
       avellanados sin interferir con el motor TT ni con el tabique guardabarros.
     - Torretas del puente de bateria P05 desplazadas a X = +-52.0 mm fuera del
       alojamiento del BMS (X = 43.0 mm) y apilamiento Z con holguras reales.
     - Reubicacion de Interruptor KCD1 (X = 56.0, Y = -68.0 mm), Portafusible 5A
       (X = 64.0, Y = 44.0 mm) y arcos/bridas sujetacables (Y = -3.8 mm).
     - Vaciado real de neumaticos/rines, cajeado de tornillos de eje y cavidad
       esferica en Ball Caster + espaciador P06 + suelo de chasis P01.
     - Ruteo tridimensional de los 8 cables NURBS libre de colisiones.
  4. Motor automatico de comprobacion de colisiones y holguras (depsgraph +
     AABB + BVHTree.FromBMesh + volumen de interseccion booleana EXACT).
================================================================================
"""

import math
import os
import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# ==============================================================================
# 1. PARAMETROS MAESTROS Y HOLGURAS CONFIGURABLES DFM/DFA (MILIMETROS)
# ==============================================================================
PARAMS = {
    "LIMITE_DIM_MAX": 200.0,
    "LIMITE_MASA_MAX_G": 750.0,
    "OBJETIVO_MASA_ING_G": 650.0,
    "DIAM_BALON": 35.50,
    "LIMITE_CAVIDAD_MM": 10.65,
    "ANCHO_CHASIS": 165.0,
    "LARGO_CHASIS": 170.0,
    "DESPEJE_SUELO_Z": 14.0,
    "ALTO_PARED_CHASIS": 46.0,
    "ESPESOR_SUELO": 3.5,
    "ESPESOR_PARED": 4.0,
    "CHAFLAN_ESQUINAS": 8.0,
    "ALTO_TAPA": 3.5,
    "ALTO_LABIO_TAPA": 2.0,
    # --- HOLGURAS CONFIGURABLES PARA IMPRESION 3D (DFM/DFA) ---
    "HOLGURA_TAPA_ENC": (
        0.40
    ),  # Ajuste deslizante labio perimetral tapa - chasis
    "HOLGURA_RUEDA_RADIAL": (
        2.50
    ),  # Espacio libre radial/superior para barrido de giro de llanta
    "HOLGURA_RUEDA_AXIAL": (
        1.80
    ),  # Espacio libre lateral de cada llanta dentro de su cajera
    "HOLGURA_SOPORTE_MOTOR": (
        0.30
    ),  # Ajuste interior de abrazadera en U sobre motor TT
    "HOLGURA_EJE_PASO": 0.25,  # Ajuste eje motor TT en maza de rin amarillo
    "HOLGURA_ALOJAMIENTO_COMP": (
        0.40
    ),  # Holgura de cuna portabaterias, bolsillo BMS y calado switch
    "HOLGURA_ASIENTO_Z": (
        0.20
    ),  # Separacion axial de montaje entre caras apoyadas
    "HOLGURA_TORNILLO_LIBRE": (
        0.20
    ),  # Holgura radial para paso libre de tornilleria M3
    "ESPESOR_MIN_PARED": (
        2.40
    ),  # Espesor minimo estructural en tabiques y soportes
    "PROF_CAVIDAD_DISENO": 8.50,
    "ANCHO_CAVIDAD_FRONTAL": 64.0,
    "DENSIDAD_PLA_G_CM3": 1.24,
    "FACTOR_MASA_EFECTIVA_3D": 0.52,
    "DIAM_TALADRO_M3_LIBRE": 3.40,
    "DIAM_TALADRO_M3_ROSCADO": 2.85,
    "ANCHO_CARAS_TUERCA_M3": 5.80,
    "ALTO_TUERCA_M3": 2.60,
    # --- TREN MOTRIZ Y RUEDAS ---
    "X_CENTRO_RUEDA": (
        64.5
    ),  # Posicion X de llantas para despejar guardabarros y abrazaderas
    "X_CENTRO_MOTOR": (
        32.5
    ),  # Posicion X de motores TT aislada de tabique y postes PCB
    "Y_EJE_MOTORES": -20.0,
    "Z_EJE_MOTORES": 32.5,
    "MOTOR_TT_LARGO": 70.0,
    "MOTOR_TT_ANCHO": 18.8,
    "MOTOR_TT_ALTO": 22.5,
    "RUEDA_DIAM": 65.0,
    "RUEDA_ANCHO": 26.0,
    "Y_BALL_CASTER": 60.0,
    "ALTO_NOMINAL_CASTER": 10.0,
    "ALTO_ESPACIADOR_CASTER": 4.0,
    # --- SISTEMA DE BATERIA 2S + BMS ---
    "Y_CENTRO_BATERIA": 25.0,
    "PORTABAT_ANCHO_X": 76.0,
    "PORTABAT_LARGO_Y": 40.5,
    "PORTABAT_ALTO_Z": 21.0,
    "ESPESOR_PAD_EVA_INF": 2.0,
    "ESPESOR_PAD_EVA_SUP": 1.5,
    "ESPESOR_MURO_CUNA": 3.0,
    "X_TORRETAS_PUENTE_BAT": (
        52.0
    ),  # Torretas P05 en X = +-52.0 mm fuera del bolsillo BMS
    "X_BOLSILLO_BMS": 43.0,
    # --- ELECTRONICA Y PLACA PORTADORA PCB ---
    "Y_CENTRO_PCB": -48.0,
    "PCB_ANCHO_X": 70.0,
    "PCB_LARGO_Y": 55.0,
    "PCB_ESPESOR": 1.6,
    "ALTO_POSTES_PCB": (
        27.2
    ),  # Elevado para situar la PCB sobre motores TT y abrazaderas
    "X_POSTES_PCB": 14.5,  # Postes en el pasillo central libre entre motores TT
    "X_INTERRUPTOR_KCD1": (
        56.0
    ),  # Esquina trasera derecha detras de la cajera de rueda
    "Y_INTERRUPTOR_KCD1": -68.0,
    "X_PORTAFUSIBLE": 64.0,  # Bahia delantera derecha despejada
    "Y_PORTAFUSIBLE": 44.0,
    "RENDERIZAR_PNG_VISTAS": False,  # True en PC local para exportar los 6 PNG
}

OUTPUT_DIR = os.path.join(os.path.expanduser("~"), "RoboGol_SD20262_CAD_100")
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ==============================================================================
# 2. ESCENA, MATERIALES Y LAS 7 COLECCIONES
# ==============================================================================
def inicializar_escena():
  if bpy.context.object and bpy.context.object.mode != "OBJECT":
    bpy.ops.object.mode_set(mode="OBJECT")
  bpy.ops.object.select_all(action="SELECT")
  bpy.ops.object.delete(use_global=False)

  for bloque in list(bpy.data.collections):
    bpy.data.collections.remove(bloque)
  for bloque in list(bpy.data.meshes):
    bpy.data.meshes.remove(bloque)
  for bloque in list(bpy.data.curves):
    bpy.data.curves.remove(bloque)
  for bloque in list(bpy.data.materials):
    bpy.data.materials.remove(bloque)
  for bloque in list(bpy.data.cameras):
    bpy.data.cameras.remove(bloque)
  for bloque in list(bpy.data.lights):
    bpy.data.lights.remove(bloque)

  scn = bpy.context.scene
  scn.unit_settings.system = "METRIC"
  scn.unit_settings.scale_length = 0.001
  scn.unit_settings.length_unit = "MILLIMETERS"
  scn.frame_start = 1
  scn.frame_end = 6
  scn.frame_current = 1

  nombres_colecciones = [
      "01_Carcasa_Exterior",
      "02_Tapa",
      "03_Componentes_Electronicos",
      "04_Portabaterias_y_Bateria",
      "05_Soportes_y_Fijaciones",
      "06_Cableado",
      "07_Elementos_Auxiliares",
  ]
  cols = {}
  for nombre in nombres_colecciones:
    c = bpy.data.collections.new(nombre)
    scn.collection.children.link(c)
    cols[nombre] = c
  return cols


COLS = inicializar_escena()


def mover_a_coleccion(obj, nombre_col):
  for c in list(obj.users_collection):
    c.objects.unlink(obj)
  COLS[nombre_col].objects.link(obj)


def crear_material(nombre, rgba, metalico=0.1, rugosidad=0.4):
  mat = bpy.data.materials.new(name=nombre)
  mat.use_nodes = True
  bsdf = mat.node_tree.nodes.get("Principled BSDF")
  if bsdf:
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Metallic"].default_value = metalico
    bsdf.inputs["Roughness"].default_value = rugosidad
  mat.diffuse_color = rgba
  return mat


MATS = {
    "CHASIS": crear_material(
        "MAT_PLA_Azul_Rey", (0.05, 0.22, 0.58, 1.0), 0.1, 0.35
    ),
    "TAPA": crear_material(
        "MAT_PLA_Gris_Espacial", (0.20, 0.22, 0.25, 1.0), 0.15, 0.4
    ),
    "SOPORTE": crear_material(
        "MAT_PLA_Naranja_Ing", (0.95, 0.35, 0.03, 1.0), 0.1, 0.35
    ),
    "MOTOR_AMARILLO": crear_material(
        "MAT_Maker_Amarillo_TT", (0.96, 0.78, 0.02, 1.0), 0.05, 0.3
    ),
    "EJE_BLANCO": crear_material(
        "MAT_Nylon_Blanco", (0.92, 0.92, 0.90, 1.0), 0.0, 0.4
    ),
    "CAUCHO": crear_material(
        "MAT_Caucho_Neumatico", (0.05, 0.05, 0.05, 1.0), 0.0, 0.88
    ),
    "PCB_VERDE": crear_material(
        "MAT_PCB_Verde_Soldermask", (0.02, 0.40, 0.18, 1.0), 0.25, 0.3
    ),
    "PCB_NEGRA": crear_material(
        "MAT_PCB_ESP32_Negra", (0.08, 0.09, 0.11, 1.0), 0.3, 0.3
    ),
    "PCB_ROJA": crear_material(
        "MAT_PCB_TB6612_Roja", (0.72, 0.06, 0.06, 1.0), 0.25, 0.3
    ),
    "PCB_AZUL": crear_material(
        "MAT_PCB_LM2596_Azul", (0.04, 0.28, 0.68, 1.0), 0.25, 0.3
    ),
    "CHIP_IC": crear_material(
        "MAT_Encapsulado_Epoxi_IC", (0.04, 0.04, 0.04, 1.0), 0.1, 0.6
    ),
    "COBRE_BOBINA": crear_material(
        "MAT_Cobre_Esmaltado", (0.85, 0.42, 0.15, 1.0), 0.85, 0.25
    ),
    "BRONCE": crear_material(
        "MAT_Laton_Bronce", (0.88, 0.72, 0.20, 1.0), 0.85, 0.2
    ),
    "ACERO": crear_material(
        "MAT_Acero_Inoxidable", (0.75, 0.77, 0.80, 1.0), 0.88, 0.18
    ),
    "CELDA_18650": crear_material(
        "MAT_Celda_Samsung_25R", (0.78, 0.15, 0.48, 1.0), 0.35, 0.25
    ),
    "EVA": crear_material(
        "MAT_Espuma_EVA_Negra", (0.12, 0.12, 0.14, 1.0), 0.0, 0.92
    ),
    "LED_ENCENDIDO": crear_material(
        "MAT_LED_Verde_Brillante", (0.0, 1.0, 0.22, 1.0), 0.0, 0.1
    ),
    "SW_ROJO": crear_material(
        "MAT_Switch_Actuador_Rojo", (0.85, 0.08, 0.08, 1.0), 0.05, 0.3
    ),
    "CABLE_POS": crear_material(
        "MAT_Cable_Rojo_20AWG", (0.90, 0.06, 0.06, 1.0), 0.0, 0.5
    ),
    "CABLE_GND": crear_material(
        "MAT_Cable_Negro_20AWG", (0.07, 0.07, 0.07, 1.0), 0.0, 0.5
    ),
    "CABLE_PWM": crear_material(
        "MAT_Cable_Azul_Senal", (0.08, 0.45, 0.92, 1.0), 0.0, 0.5
    ),
    "CABLE_MOT": crear_material(
        "MAT_Cable_Amarillo_Motor", (0.95, 0.68, 0.04, 1.0), 0.0, 0.5
    ),
    "BALON": crear_material(
        "MAT_Balon_Blanco_Oficial", (0.96, 0.96, 0.96, 1.0), 0.0, 0.2
    ),
    "GALGA_LIMITE": crear_material(
        "MAT_Galga_Roja_10_65mm", (1.0, 0.08, 0.08, 0.42), 0.0, 0.5
    ),
    "TEXTO_LEYENDA": crear_material(
        "MAT_Texto_Blanco_UI", (1.0, 0.95, 0.50, 1.0), 0.0, 0.2
    ),
}


# ==============================================================================
# 3. PRIMITIVAS Y BOOLEANAS
# ==============================================================================
def booleana(obj_base, obj_corte, operacion="DIFFERENCE"):
  mod = obj_base.modifiers.new(name=f"Bool_{operacion}", type="BOOLEAN")
  mod.operation = operacion
  mod.solver = "EXACT"
  mod.object = obj_corte
  bpy.ops.object.select_all(action="DESELECT")
  obj_base.select_set(True)
  bpy.context.view_layer.objects.active = obj_base
  bpy.ops.object.modifier_apply(modifier=mod.name)
  bpy.data.objects.remove(obj_corte, do_unlink=True)


def crear_caja(nombre, dim_xyz, pos_xyz, rot_deg=(0, 0, 0), mat=None, col=None):
  bpy.ops.mesh.primitive_cube_add(size=1.0, location=pos_xyz)
  obj = bpy.context.active_object
  obj.name = nombre
  obj.scale = dim_xyz
  obj.rotation_euler = tuple(math.radians(a) for a in rot_deg)
  bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
  if mat:
    obj.data.materials.append(mat)
  if col:
    mover_a_coleccion(obj, col)
  return obj


def crear_cilindro(
    nombre, radio, alto, pos_xyz, rot_deg=(0, 0, 0), segs=36, mat=None, col=None
):
  bpy.ops.mesh.primitive_cylinder_add(
      vertices=segs,
      radius=radio,
      depth=alto,
      location=pos_xyz,
      rotation=tuple(math.radians(a) for a in rot_deg),
  )
  obj = bpy.context.active_object
  obj.name = nombre
  bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
  if mat:
    obj.data.materials.append(mat)
  if col:
    mover_a_coleccion(obj, col)
  return obj


def unir_objetos_en_uno(obj_principal, lista_secundarios):
  bpy.ops.object.select_all(action="DESELECT")
  obj_principal.select_set(True)
  for s in lista_secundarios:
    s.select_set(True)
  bpy.context.view_layer.objects.active = obj_principal
  bpy.ops.object.join()
  return obj_principal


# ==============================================================================
# 4. PIEZA P01: CHASIS BASE (COLECCION 01)
# ==============================================================================
def construir_p01_chasis():
  W = PARAMS["ANCHO_CHASIS"]
  L = PARAMS["LARGO_CHASIS"]
  Z0 = PARAMS["DESPEJE_SUELO_Z"]
  H = PARAMS["ALTO_PARED_CHASIS"]
  Ts = PARAMS["ESPESOR_SUELO"]
  Tp = PARAMS["ESPESOR_PARED"]
  Z_piso = Z0 + Ts

  chasis = crear_caja(
      "P01_Chasis_Base_Estructural",
      (W, L, H),
      (0.0, 0.0, Z0 + H / 2.0),
      mat=MATS["CHASIS"],
      col="01_Carcasa_Exterior",
  )

  # 4.1 Chaflanes exteriores a 45 grados
  chaf = PARAMS["CHAFLAN_ESQUINAS"]
  for sx in [-1.0, 1.0]:
    for sy in [-1.0, 1.0]:
      c_esq = crear_caja(
          "Cut_Chaflan",
          (chaf * 2.0, chaf * 2.0, H + 6.0),
          (sx * (W / 2.0), sy * (L / 2.0), Z0 + H / 2.0),
          rot_deg=(0, 0, 45),
      )
      booleana(chasis, c_esq, "DIFFERENCE")

  # 4.2 Vaciado interior (con chaflanes internos a 45 grados para conservar espesor de pared en esquinas)
  largo_int = L - 2.0 * Tp - 10.5
  ancho_int = W - 2.0 * Tp
  alto_int = H - Ts + 2.0
  y_int = -5.25
  vaciado = crear_caja(
      "Cut_Vaciado_Interior",
      (ancho_int, largo_int, alto_int),
      (0.0, y_int, Z_piso + alto_int / 2.0),
  )
  for sx in [-1.0, 1.0]:
    for sy_pos in [y_int - largo_int / 2.0, y_int + largo_int / 2.0]:
      c_esq_int = crear_caja(
          "Cut_Chaflan_Vaciado",
          (chaf * 2.2, chaf * 2.2, alto_int + 4.0),
          (sx * (ancho_int / 2.0), sy_pos, Z_piso + alto_int / 2.0),
          rot_deg=(0, 0, 45),
      )
      booleana(vaciado, c_esq_int, "DIFFERENCE")
  booleana(chasis, vaciado, "DIFFERENCE")

  # 4.3 Cavidad frontal concava para balon reglamentario
  d_cav = PARAMS["PROF_CAVIDAD_DISENO"]
  w_cav = PARAMS["ANCHO_CAVIDAD_FRONTAL"]
  r_corte = (d_cav / 2.0) + ((w_cav**2) / (8.0 * d_cav))
  y_centro_corte = (L / 2.0) - d_cav + r_corte
  cil_pala = crear_cilindro(
      "Cut_Cavidad_Balon_8_5mm",
      r_corte,
      H + 6.0,
      (0.0, y_centro_corte, Z0 + H / 2.0),
      segs=96,
  )
  booleana(chasis, cil_pala, "DIFFERENCE")

  # 4.4 Cajeras de ruedas (abiertas arriba hasta Z > 60 mm), guardabarros, pasos de eje y guias de motor
  y_eje = PARAMS["Y_EJE_MOTORES"]
  z_eje = PARAMS["Z_EJE_MOTORES"]
  xr_abs = PARAMS["X_CENTRO_RUEDA"]
  xm_abs = PARAMS["X_CENTRO_MOTOR"]
  ancho_cajera = PARAMS["RUEDA_ANCHO"] + 2.0 * PARAMS["HOLGURA_RUEDA_AXIAL"]
  largo_cajera = (
      PARAMS["RUEDA_DIAM"] + 2.0 * PARAMS["HOLGURA_RUEDA_RADIAL"] + 1.0
  )
  espesor_tabique = 2.8
  x_tabique_abs = xr_abs - ancho_cajera / 2.0 - espesor_tabique / 2.0

  for sx in [-1.0, 1.0]:
    x_rueda = sx * xr_abs
    caj_rueda = crear_caja(
        "Cut_Cajera_Rueda",
        (ancho_cajera, largo_cajera, H + 8.0),
        (x_rueda, y_eje, Z0 + H / 2.0),
    )
    booleana(chasis, caj_rueda, "DIFFERENCE")

    tabique_int = crear_caja(
        "Union_Tabique_Guardabarros",
        (espesor_tabique, largo_cajera + 3.0, H - Ts),
        (sx * x_tabique_abs, y_eje, Z_piso + (H - Ts) / 2.0),
    )
    booleana(chasis, tabique_int, "UNION")

    ventana_eje = crear_cilindro(
        "Cut_Paso_Eje_TT",
        7.5,
        16.0,
        (sx * x_tabique_abs, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=32,
    )
    booleana(chasis, ventana_eje, "DIFFERENCE")

    # Guia lateral trasera del motor (detras de la abrazadera P03/P04)
    x_guia = sx * (
        xm_abs
        - PARAMS["MOTOR_TT_ANCHO"] / 2.0
        - PARAMS["HOLGURA_SOPORTE_MOTOR"]
        - 1.2
    )
    guia_mot_int = crear_caja(
        "Union_Guia_Motor_Int", (2.4, 20.0, 4.5), (x_guia, -50.5, Z_piso + 2.25)
    )
    booleana(chasis, guia_mot_int, "UNION")

    # Tope trasero del motor detras del cilindro DC (Y = -74.5 mm)
    tope_mot_tras = crear_caja(
        "Union_Tope_Trasero_Motor",
        (14.0, 2.5, 5.0),
        (sx * xm_abs, -74.5, Z_piso + 2.5),
    )
    booleana(chasis, tope_mot_tras, "UNION")

  # 4.5 Nervaduras frontales anti-impacto
  for x_nerv in [-27.0, -9.0, 9.0, 27.0]:
    nerv = crear_caja(
        "Union_Nervadura_Impacto",
        (3.5, 14.0, 26.0),
        (x_nerv, (L / 2.0) - 17.5, Z_piso + 13.0),
    )
    booleana(chasis, nerv, "UNION")

  # 4.6 Cuna de bateria, torretas P05 (X = +-52.0 mm), alojamiento portabaterias y bolsillo BMS
  yb = PARAMS["Y_CENTRO_BATERIA"]
  holg_c = PARAMS["HOLGURA_ALOJAMIENTO_COMP"]
  wb_int = PARAMS["PORTABAT_ANCHO_X"] + 2.0 * holg_c
  lb_int = PARAMS["PORTABAT_LARGO_Y"] + 2.0 * holg_c
  tc = PARAMS["ESPESOR_MURO_CUNA"]
  hc = 13.0
  x_bms = PARAMS["X_BOLSILLO_BMS"]
  x_torreta_bat = PARAMS["X_TORRETAS_PUENTE_BAT"]

  cuna_bloque = crear_caja(
      "Union_Cuna_Bateria",
      (wb_int + 2.0 * tc + 8.0, lb_int + 2.0 * tc, hc),
      (4.0, yb, Z_piso + hc / 2.0),
  )
  booleana(chasis, cuna_bloque, "UNION")

  # Torretas P05 unidas antes de los vaciados para preservar la cavidad del BMS
  h_torreta_p05 = (
      PARAMS["ESPESOR_PAD_EVA_INF"]
      + PARAMS["PORTABAT_ALTO_Z"]
      + PARAMS["ESPESOR_PAD_EVA_SUP"]
      + 2.0 * PARAMS["HOLGURA_ASIENTO_Z"]
  )
  for sx in [-1.0, 1.0]:
    torreta_pb = crear_cilindro(
        "Union_Torreta_Puente_P05",
        5.2,
        h_torreta_p05,
        (sx * x_torreta_bat, yb, Z_piso + h_torreta_p05 / 2.0),
        segs=24,
    )
    booleana(chasis, torreta_pb, "UNION")

  cuna_hueco = crear_caja(
      "Cut_Alojamiento_Portabat",
      (wb_int, lb_int, hc + 4.0),
      (0.0, yb, Z_piso + hc / 2.0 + 2.0),
  )
  booleana(chasis, cuna_hueco, "DIFFERENCE")

  paso_dedos = crear_caja(
      "Cut_Ranura_Dedos_Extraccion",
      (24.0, lb_int + 12.0, hc - 3.0),
      (0.0, yb, Z_piso + (hc - 3.0) / 2.0 + 3.5),
  )
  booleana(chasis, paso_dedos, "DIFFERENCE")

  bolsillo_bms = crear_caja(
      "Cut_Bolsillo_BMS_2S",
      (6.8, 38.0, hc + 4.0),
      (x_bms, yb, Z_piso + hc / 2.0 + 2.0),
  )
  booleana(chasis, bolsillo_bms, "DIFFERENCE")

  canal_cables_bms = crear_caja(
      "Cut_Canal_Salida_Cables_BMS",
      (10.0, 14.0, hc + 2.0),
      (x_bms, yb - lb_int / 2.0 - tc, Z_piso + hc / 2.0 + 2.5),
  )
  booleana(chasis, canal_cables_bms, "DIFFERENCE")

  # 4.7 Postes PCB (en el pasillo central libre entre motores TT)
  yp = PARAMS["Y_CENTRO_PCB"]
  hp = PARAMS["ALTO_POSTES_PCB"]
  xp_pcb = PARAMS["X_POSTES_PCB"]
  coords_postes_pcb = [
      (-xp_pcb, yp + 23.5),
      (xp_pcb, yp + 23.5),
      (-xp_pcb, yp - 23.5),
      (xp_pcb, yp - 23.5),
  ]
  for px, py in coords_postes_pcb:
    poste = crear_cilindro(
        "Union_Poste_PCB", 3.4, hp, (px, py, Z_piso + hp / 2.0), segs=24
    )
    booleana(chasis, poste, "UNION")
    tal_p = crear_cilindro(
        "Cut_Rosca_Poste_PCB",
        PARAMS["DIAM_TALADRO_M3_ROSCADO"] / 2.0,
        hp + 2.0,
        (px, py, Z_piso + hp / 2.0 + 1.0),
        segs=20,
    )
    booleana(chasis, tal_p, "DIFFERENCE")

  # 4.8 Columnas esquineras para la tapa (remetidas dentro del chaflan exterior)
  coords_columnas_tapa = [
      (-72.5, 72.0),
      (72.5, 72.0),
      (-72.5, -73.0),
      (72.5, -73.0),
  ]
  h_col = H - Ts
  r_hex = PARAMS["ANCHO_CARAS_TUERCA_M3"] / math.sqrt(3.0)
  for cx, cy in coords_columnas_tapa:
    col_tapa = crear_cilindro(
        "Union_Columna_Tapa", 5.5, h_col, (cx, cy, Z_piso + h_col / 2.0), segs=24
    )
    booleana(chasis, col_tapa, "UNION")
    tal_col = crear_cilindro(
        "Cut_Paso_M3_Columna",
        PARAMS["DIAM_TALADRO_M3_LIBRE"] / 2.0,
        22.0,
        (cx, cy, Z0 + H - 9.0),
        segs=20,
    )
    booleana(chasis, tal_col, "DIFFERENCE")
    nut_col = crear_cilindro(
        "Cut_NutTrap_Columna",
        r_hex,
        PARAMS["ALTO_TUERCA_M3"] + 0.6,
        (cx, cy, Z0 + H - 6.0),
        segs=6,
    )
    booleana(chasis, nut_col, "DIFFERENCE")

  # 4.9 Arcos sujetacables en el pasillo central (Y = -3.8 mm, fuera de cuna y motores)
  for x_arco in [-15.0, -5.0, 5.0, 15.0]:
    arco = crear_caja(
        "Union_Arco_Cable", (6.4, 4.4, 4.5), (x_arco, -3.8, Z_piso + 2.25)
    )
    booleana(chasis, arco, "UNION")
    hueco_arco = crear_caja(
        "Cut_Paso_Brida", (3.8, 7.0, 2.5), (x_arco, -3.8, Z_piso + 1.25)
    )
    booleana(chasis, hueco_arco, "DIFFERENCE")

  # 4.10 Ventana trasera USB alineada con la cota Z del ESP32
  z_esp_usb = (
      Z_piso
      + hp
      + PARAMS["HOLGURA_ASIENTO_Z"]
      + PARAMS["PCB_ESPESOR"]
      + 5.2
      + 2.0
  )
  vent_usb = crear_caja(
      "Cut_Ventana_USB_Trasera", (16.0, 12.0, 9.0), (0.0, -L / 2.0, z_esp_usb)
  )
  booleana(chasis, vent_usb, "DIFFERENCE")

  # 4.11 Cajeado inferior para despeje de la esfera del Ball Caster
  yc = PARAMS["Y_BALL_CASTER"]
  despeje_esfera_caster = crear_cilindro(
      "Cut_Despeje_Esfera_Caster", 8.3, 4.0, (0.0, yc, Z0 + 0.5), segs=32
  )
  booleana(chasis, despeje_esfera_caster, "DIFFERENCE")

  # 4.12 Taladros M3 en el suelo + nut traps
  dx_or = 11.4
  ym_torn = -29.0
  taladros_suelo_m3 = [
      (-14.0, yc),
      (14.0, yc),
      (-22.0, yb),
      (22.0, yb),
      (-xm_abs + dx_or, ym_torn),
      (-xm_abs - dx_or, ym_torn),
      (xm_abs - dx_or, ym_torn),
      (xm_abs + dx_or, ym_torn),
      (-x_torreta_bat, yb),
      (x_torreta_bat, yb),
  ]
  r_m3 = PARAMS["DIAM_TALADRO_M3_LIBRE"] / 2.0
  for tx, ty in taladros_suelo_m3:
    es_torreta_bat = abs(ty - yb) < 1.0 and abs(tx) > 40.0
    h_corte = 70.0 if es_torreta_bat else 36.0
    z_corte = (Z_piso + 20.0) if es_torreta_bat else (Z_piso + 8.0)
    cil_m3 = crear_cilindro(
        "Cut_Taladro_M3", r_m3, h_corte, (tx, ty, z_corte), segs=20
    )
    booleana(chasis, cil_m3, "DIFFERENCE")
    nut_suelo = crear_cilindro(
        "Cut_NutTrap_Suelo",
        r_hex,
        PARAMS["ALTO_TUERCA_M3"] + 0.6,
        (tx, ty, Z0 + 1.3),
        segs=6,
    )
    booleana(chasis, nut_suelo, "DIFFERENCE")

  return chasis, coords_postes_pcb, coords_columnas_tapa, taladros_suelo_m3


# ==============================================================================
# 5. PIEZA P02: TAPA SUPERIOR (COLECCION 02)
# ==============================================================================
def construir_p02_tapa(coords_columnas_tapa):
  W = PARAMS["ANCHO_CHASIS"]
  L = PARAMS["LARGO_CHASIS"]
  Z_base = (
      PARAMS["DESPEJE_SUELO_Z"]
      + PARAMS["ALTO_PARED_CHASIS"]
      + PARAMS["HOLGURA_ASIENTO_Z"]
  )
  Ht = PARAMS["ALTO_TAPA"]
  Tp = PARAMS["ESPESOR_PARED"]
  holg_tapa = PARAMS["HOLGURA_TAPA_ENC"]

  tapa = crear_caja(
      "P02_Tapa_Superior_Protectora",
      (W, L - 8.0, Ht),
      (0.0, -4.0, Z_base + Ht / 2.0),
      mat=MATS["TAPA"],
      col="02_Tapa",
  )

  chaf = PARAMS["CHAFLAN_ESQUINAS"]
  for sx in [-1.0, 1.0]:
    for sy in [-1.0, 1.0]:
      c_esq = crear_caja(
          "Cut_Chaflan_Tapa",
          (chaf * 2.0, chaf * 2.0, Ht + 6.0),
          (sx * (W / 2.0), sy * (L / 2.0), Z_base + Ht / 2.0),
          rot_deg=(0, 0, 45),
      )
      booleana(tapa, c_esq, "DIFFERENCE")

  # Labio perimetral inferior con chaflanes en esquinas y despeje de columnas esquineras
  ancho_labio_ext = W - 2.0 * Tp - 2.0 * holg_tapa
  largo_labio_ext = L - 22.0
  labio_ext = crear_caja(
      "Union_Labio_Centrado_Tapa",
      (ancho_labio_ext, largo_labio_ext, PARAMS["ALTO_LABIO_TAPA"]),
      (0.0, -6.0, Z_base - PARAMS["ALTO_LABIO_TAPA"] / 2.0),
  )
  labio_int = crear_caja(
      "Cut_Interior_Labio",
      (
          ancho_labio_ext - 4.0,
          largo_labio_ext - 4.0,
          PARAMS["ALTO_LABIO_TAPA"] + 2.0,
      ),
      (0.0, -6.0, Z_base - PARAMS["ALTO_LABIO_TAPA"] / 2.0),
  )
  booleana(labio_ext, labio_int, "DIFFERENCE")

  for cx, cy in coords_columnas_tapa:
    c_col_lab = crear_cilindro(
        "Cut_Despeje_Columna_Labio",
        6.2 + holg_tapa,
        PARAMS["ALTO_LABIO_TAPA"] + 2.0,
        (cx, cy, Z_base - PARAMS["ALTO_LABIO_TAPA"] / 2.0),
        segs=24,
    )
    booleana(labio_ext, c_col_lab, "DIFFERENCE")

  for sx in [-1.0, 1.0]:
    for sy_lab in [-6.0 - largo_labio_ext / 2.0, -6.0 + largo_labio_ext / 2.0]:
      c_esq_lab = crear_caja(
          "Cut_Chaflan_Labio",
          (chaf * 2.4, chaf * 2.4, PARAMS["ALTO_LABIO_TAPA"] + 4.0),
          (
              sx * (ancho_labio_ext / 2.0),
              sy_lab,
              Z_base - PARAMS["ALTO_LABIO_TAPA"] / 2.0,
          ),
          rot_deg=(0, 0, 45),
      )
      booleana(labio_ext, c_esq_lab, "DIFFERENCE")

  booleana(tapa, labio_ext, "UNION")

  # Recortes de paso superior y despeje de labio para las llantas de Ø65 mm
  xr_abs = PARAMS["X_CENTRO_RUEDA"]
  y_eje = PARAMS["Y_EJE_MOTORES"]
  ancho_cajera_tapa = PARAMS["RUEDA_ANCHO"] + 2.0 * PARAMS["HOLGURA_RUEDA_AXIAL"]
  largo_cajera_tapa = (
      PARAMS["RUEDA_DIAM"] + 2.0 * PARAMS["HOLGURA_RUEDA_RADIAL"] + 1.0
  )
  for sx in [-1.0, 1.0]:
    corte_labio_rueda = crear_caja(
        "Cut_Despeje_Labio_Guardabarros",
        (40.0, largo_cajera_tapa + 6.0, PARAMS["ALTO_LABIO_TAPA"] + 2.0),
        (sx * 64.0, y_eje, Z_base - PARAMS["ALTO_LABIO_TAPA"] / 2.0),
    )
    booleana(tapa, corte_labio_rueda, "DIFFERENCE")

    corte_rueda_tapa = crear_caja(
        "Cut_Abertura_Rueda_Tapa",
        (
            ancho_cajera_tapa,
            largo_cajera_tapa,
            Ht + PARAMS["ALTO_LABIO_TAPA"] + 6.0,
        ),
        (sx * xr_abs, y_eje, Z_base),
    )
    booleana(tapa, corte_rueda_tapa, "DIFFERENCE")

  # Domo del LED con alojamiento para el bisel del portaled (r = 4.5 mm)
  domo = crear_cilindro(
      "Union_Torreta_LED",
      12.0,
      11.5,
      (0.0, -55.0, Z_base + Ht + 5.75),
      segs=32,
  )
  booleana(tapa, domo, "UNION")
  tal_led = crear_cilindro(
      "Cut_Alojamiento_Portaled",
      4.5,
      26.0,
      (0.0, -55.0, Z_base + 7.0),
      segs=24,
  )
  booleana(tapa, tal_led, "DIFFERENCE")

  # Calado del interruptor rocker KCD1 en su posicion libre (X = 56.0, Y = -68.0)
  x_sw = PARAMS["X_INTERRUPTOR_KCD1"]
  y_sw = PARAMS["Y_INTERRUPTOR_KCD1"]
  calado_sw = crear_caja(
      "Cut_Calado_Interruptor_KCD1",
      (
          13.0 + 2.0 * PARAMS["HOLGURA_ALOJAMIENTO_COMP"],
          19.0 + 2.0 * PARAMS["HOLGURA_ALOJAMIENTO_COMP"],
          16.0,
      ),
      (x_sw, y_sw, Z_base + Ht / 2.0),
  )
  booleana(tapa, calado_sw, "DIFFERENCE")

  # Ventana de acceso al jumper RF
  vent_jumper = crear_caja(
      "Cut_Acceso_Jumper_RF",
      (10.0, 10.0, 14.0),
      (-9.0, -24.0, Z_base + Ht / 2.0),
  )
  booleana(tapa, vent_jumper, "DIFFERENCE")

  # Ranuras de ventilacion
  for i in range(-2, 3):
    ranura = crear_caja(
        "Cut_Ranura_Ventilacion",
        (5.0, 42.0, 14.0),
        (i * 13.0, 10.0, Z_base + Ht / 2.0),
    )
    booleana(tapa, ranura, "DIFFERENCE")

  # Orificios M3 avellanados en las 4 esquinas
  for cx, cy in coords_columnas_tapa:
    tal_m3 = crear_cilindro(
        "Cut_M3_Tapa",
        PARAMS["DIAM_TALADRO_M3_LIBRE"] / 2.0,
        16.0,
        (cx, cy, Z_base + Ht / 2.0),
        segs=20,
    )
    booleana(tapa, tal_m3, "DIFFERENCE")
    avellanado = crear_cilindro(
        "Cut_Avellanado_M3", 3.3, 2.2, (cx, cy, Z_base + Ht - 0.5), segs=20
    )
    booleana(tapa, avellanado, "DIFFERENCE")

  return tapa


# ==============================================================================
# 6. SOPORTES (P03-P06) Y TORNILLERIA (COLECCION 05)
# ==============================================================================
def crear_tornillo_m3_detallado(
    nombre,
    largo_vastago,
    pos_xyz,
    rot_deg=(0, 0, 0),
    avellanado=False,
    radio_vastago=1.45,
):
  vastago = crear_cilindro(
      f"{nombre}_Vastago",
      radio_vastago,
      largo_vastago,
      (pos_xyz[0], pos_xyz[1], pos_xyz[2] - largo_vastago / 2.0),
      segs=16,
      mat=MATS["ACERO"],
  )
  r_cab = 2.9 if avellanado else 2.65
  h_cab = 1.2 if avellanado else 2.8
  cabeza = crear_cilindro(
      f"{nombre}_Cabeza",
      r_cab,
      h_cab,
      (pos_xyz[0], pos_xyz[1], pos_xyz[2] + h_cab / 2.0),
      segs=24,
      mat=MATS["ACERO"],
  )
  hex_allen = crear_cilindro(
      "Cut_Hex_Allen",
      1.25,
      1.5,
      (pos_xyz[0], pos_xyz[1], pos_xyz[2] + h_cab - 0.4),
      segs=6,
  )
  booleana(cabeza, hex_allen, "DIFFERENCE")
  tornillo = unir_objetos_en_uno(cabeza, [vastago])
  tornillo.name = nombre
  tornillo.rotation_euler = tuple(math.radians(a) for a in rot_deg)
  bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
  mover_a_coleccion(tornillo, "05_Soportes_y_Fijaciones")
  return tornillo


def crear_tuerca_m3_detallada(nombre, pos_xyz):
  r_ext = 5.4 / math.sqrt(3.0)
  tuerca = crear_cilindro(
      nombre,
      r_ext,
      2.2,
      pos_xyz,
      segs=6,
      mat=MATS["ACERO"],
      col="05_Soportes_y_Fijaciones",
  )
  rosca = crear_cilindro("Cut_Rosca_M3", 1.55, 4.0, pos_xyz, segs=16)
  booleana(tuerca, rosca, "DIFFERENCE")
  return tuerca


def construir_soportes_y_fijaciones(
    coords_postes_pcb, coords_columnas_tapa, taladros_suelo_m3
):
  Z_piso = PARAMS["DESPEJE_SUELO_Z"] + PARAMS["ESPESOR_SUELO"]
  yb = PARAMS["Y_CENTRO_BATERIA"]
  holg_z = PARAMS["HOLGURA_ASIENTO_Z"]
  xm_abs = PARAMS["X_CENTRO_MOTOR"]
  soportes_stl = []

  # 6.1 P03 y P04: abrazaderas en U para motores TT
  z_base_abraz = Z_piso + holg_z
  h_abraz = 26.8
  h_hueco_u = (
      PARAMS["Z_EJE_MOTORES"]
      + PARAMS["MOTOR_TT_ALTO"] / 2.0
      + PARAMS["HOLGURA_SOPORTE_MOTOR"]
  ) - z_base_abraz
  w_hueco_u = PARAMS["MOTOR_TT_ANCHO"] + 2.0 * PARAMS["HOLGURA_SOPORTE_MOTOR"]
  dx_or = 11.4

  for idx, (sx, lado) in enumerate([(-1.0, "Izq"), (1.0, "Der")], start=3):
    xm = sx * xm_abs
    ym = -28.0
    ym_torn = -29.0
    abraz = crear_caja(
        f"P0{idx}_Abrazadera_Motor_{lado}",
        (27.6, 22.0, h_abraz),
        (xm, ym, z_base_abraz + h_abraz / 2.0),
        mat=MATS["SOPORTE"],
        col="05_Soportes_y_Fijaciones",
    )
    hueco_u = crear_caja(
        "Cut_U_Motor",
        (w_hueco_u, 25.0, h_hueco_u + 1.0),
        (xm, ym, z_base_abraz + (h_hueco_u - 1.0) / 2.0),
    )
    booleana(abraz, hueco_u, "DIFFERENCE")

    paso_eje_abraz = crear_cilindro(
        "Cut_Paso_Eje_Abrazadera",
        4.2,
        32.0,
        (xm, PARAMS["Y_EJE_MOTORES"], PARAMS["Z_EJE_MOTORES"]),
        rot_deg=(0, 90, 0),
        segs=24,
    )
    booleana(abraz, paso_eje_abraz, "DIFFERENCE")

    for so in [-1.0, 1.0]:
      # Ventana central de 8.0 mm en Y que conserva pilares delantero y trasero continuos
      rebaje = crear_caja(
          "Cut_Rebaje_Orejeta",
          (6.0, 8.0, 16.0),
          (xm + so * 11.8, ym_torn, z_base_abraz + 2.0 + 8.0),
      )
      booleana(abraz, rebaje, "DIFFERENCE")
      tal_or = crear_cilindro(
          "Cut_M3_Orejeta",
          PARAMS["DIAM_TALADRO_M3_LIBRE"] / 2.0,
          15.0,
          (xm + so * dx_or, ym_torn, z_base_abraz + 2.5),
          segs=20,
      )
      booleana(abraz, tal_or, "DIFFERENCE")

    perno_trans = crear_cilindro(
        "Cut_Perno_Transversal_TT",
        1.65,
        30.0,
        (xm, ym - 4.0, z_base_abraz + 11.5),
        rot_deg=(0, 90, 0),
        segs=20,
    )
    booleana(abraz, perno_trans, "DIFFERENCE")
    soportes_stl.append(abraz)

  # 6.2 P05: puente de retencion de bateria
  h_torreta_p05 = (
      PARAMS["ESPESOR_PAD_EVA_INF"]
      + PARAMS["PORTABAT_ALTO_Z"]
      + PARAMS["ESPESOR_PAD_EVA_SUP"]
      + 2.0 * holg_z
  )
  h_p05 = 5.5
  z_puente = Z_piso + h_torreta_p05 + holg_z + h_p05 / 2.0
  x_torreta_bat = PARAMS["X_TORRETAS_PUENTE_BAT"]
  puente_bat = crear_caja(
      "P05_Puente_Retencion_Bateria",
      (2.0 * x_torreta_bat + 10.0, 15.0, h_p05),
      (0.0, yb, z_puente),
      mat=MATS["SOPORTE"],
      col="05_Soportes_y_Fijaciones",
  )
  vent_alig = crear_caja(
      "Cut_Aligeramiento_P05", (42.0, 8.0, 10.0), (0.0, yb, z_puente)
  )
  booleana(puente_bat, vent_alig, "DIFFERENCE")
  for sx in [-1.0, 1.0]:
    tal_p05 = crear_cilindro(
        "Cut_M3_P05",
        PARAMS["DIAM_TALADRO_M3_LIBRE"] / 2.0,
        12.0,
        (sx * x_torreta_bat, yb, z_puente),
        segs=20,
    )
    booleana(puente_bat, tal_p05, "DIFFERENCE")
  soportes_stl.append(puente_bat)

  # 6.3 P06: espaciador del ball caster con despeje central para la esfera de Ø15 mm
  h_esp = PARAMS["ALTO_ESPACIADOR_CASTER"] - 2.0 * holg_z
  yc = PARAMS["Y_BALL_CASTER"]
  z_esp_caster = (
      PARAMS["DESPEJE_SUELO_Z"] - PARAMS["ALTO_ESPACIADOR_CASTER"] / 2.0
  )
  espaciador = crear_caja(
      "P06_Espaciador_BallCaster",
      (38.0, 22.0, h_esp),
      (0.0, yc, z_esp_caster),
      mat=MATS["SOPORTE"],
      col="05_Soportes_y_Fijaciones",
  )
  hueco_esfera_p06 = crear_cilindro(
      "Cut_Despeje_Esfera_P06", 8.3, h_esp + 4.0, (0.0, yc, z_esp_caster), segs=32
  )
  booleana(espaciador, hueco_esfera_p06, "DIFFERENCE")
  for sx in [-1.0, 1.0]:
    tal_esp = crear_cilindro(
        "Cut_M3_P06",
        PARAMS["DIAM_TALADRO_M3_LIBRE"] / 2.0,
        h_esp + 4.0,
        (sx * 14.0, yc, z_esp_caster),
        segs=20,
    )
    booleana(espaciador, tal_esp, "DIFFERENCE")
  soportes_stl.append(espaciador)

  # 6.4 Separadores de laton bajo el LM2596
  z_pcb_sup = (
      Z_piso + PARAMS["ALTO_POSTES_PCB"] + holg_z + PARAMS["PCB_ESPESOR"]
  )
  h_sep_laton = 2.0
  for idx_s, y_sep in enumerate([-59.0, -37.0], start=1):
    crear_cilindro(
        f"FIJ_Separador_Laton_LM2596_{idx_s:02d}",
        2.4,
        h_sep_laton - 2.0 * holg_z,
        (24.5, y_sep, z_pcb_sup + h_sep_laton / 2.0),
        segs=6,
        mat=MATS["BRONCE"],
        col="05_Soportes_y_Fijaciones",
    )

  # 6.5 Bridas sujetacables en Y = -3.8 mm
  for idx_b, x_arco in enumerate([-15.0, -5.0, 5.0, 15.0], start=1):
    brida = crear_caja(
        f"FIJ_Brida_Sujetacables_{idx_b:02d}",
        (3.0, 7.2, 7.2),
        (x_arco, -3.8, Z_piso + 4.2),
        mat=MATS["SW_ROJO"],
        col="05_Soportes_y_Fijaciones",
    )
    hueco_b = crear_caja(
        "Cut_Brida_Int", (6.0, 5.4, 4.6), (x_arco, -3.8, Z_piso + 4.2)
    )
    booleana(brida, hueco_b, "DIFFERENCE")

  # 6.6 Tornillos y tuercas M3
  for idx_t, (tx, ty) in enumerate(taladros_suelo_m3, start=1):
    es_portabat = abs(ty - yb) < 1.0 and abs(tx) < 30.0
    es_torreta_bat = abs(ty - yb) < 1.0 and abs(tx) > 40.0
    if es_torreta_bat:
      z_cab = z_puente + h_p05 / 2.0 + holg_z
      largo_v = 16.0
    elif es_portabat:
      z_cab = Z_piso + PARAMS["ESPESOR_PAD_EVA_INF"] + 2.2
      largo_v = 8.0
    else:
      z_cab = (z_base_abraz + 2.15) if ty < 0 else (Z_piso + holg_z)
      largo_v = 7.0 if ty < 0 else 8.0
    crear_tornillo_m3_detallado(
        f"FIJ_Tornillo_M3_Chasis_{idx_t:02d}",
        largo_v,
        (tx, ty, z_cab),
        avellanado=(es_portabat or ty < 0),
    )
    crear_tuerca_m3_detallada(
        f"FIJ_Tuerca_M3_Chasis_{idx_t:02d}",
        (tx, ty, PARAMS["DESPEJE_SUELO_Z"] + 1.3),
    )

  for idx_p, (px, py) in enumerate(coords_postes_pcb, start=1):
    crear_tornillo_m3_detallado(
        f"FIJ_Tornillo_M3_PCB_{idx_p:02d}",
        8.0,
        (px, py, z_pcb_sup + holg_z),
        radio_vastago=1.35,
    )

  Z_tapa_sup = (
      PARAMS["DESPEJE_SUELO_Z"]
      + PARAMS["ALTO_PARED_CHASIS"]
      + holg_z
      + PARAMS["ALTO_TAPA"]
  )
  for idx_c, (cx, cy) in enumerate(coords_columnas_tapa, start=1):
    crear_tornillo_m3_detallado(
        f"FIJ_Tornillo_M3_Tapa_{idx_c:02d}",
        12.0,
        (cx, cy, Z_tapa_sup - 1.2),
        avellanado=True,
    )
    crear_tuerca_m3_detallada(
        f"FIJ_Tuerca_M3_ColumnaTapa_{idx_c:02d}",
        (cx, cy, PARAMS["DESPEJE_SUELO_Z"] + PARAMS["ALTO_PARED_CHASIS"] - 6.0),
    )

  return soportes_stl


# ==============================================================================
# 7. SISTEMA DE BATERIA (COLECCION 04)
# ==============================================================================
def construir_sistema_bateria_completo():
  Z_piso = PARAMS["DESPEJE_SUELO_Z"] + PARAMS["ESPESOR_SUELO"]
  yb = PARAMS["Y_CENTRO_BATERIA"]
  holg_z = PARAMS["HOLGURA_ASIENTO_Z"]
  h_eva_inf = PARAMS["ESPESOR_PAD_EVA_INF"] - 2.0 * holg_z

  z_ctr_eva_inf = Z_piso + PARAMS["ESPESOR_PAD_EVA_INF"] / 2.0
  pad_inf = crear_caja(
      "COMP_Pad_EVA_Inferior_Bateria",
      (
          PARAMS["PORTABAT_ANCHO_X"] - 0.4,
          PARAMS["PORTABAT_LARGO_Y"] - 0.5,
          h_eva_inf,
      ),
      (0.0, yb, z_ctr_eva_inf),
      mat=MATS["EVA"],
      col="04_Portabaterias_y_Bateria",
  )
  for sx in [-1.0, 1.0]:
    t_eva = crear_cilindro(
        "Cut_M3_EVA", 2.2, 6.0, (sx * 22.0, yb, z_ctr_eva_inf), segs=20
    )
    booleana(pad_inf, t_eva, "DIFFERENCE")

  z_pb = Z_piso + PARAMS["ESPESOR_PAD_EVA_INF"] + holg_z
  h_pb = PARAMS["PORTABAT_ALTO_Z"] - holg_z
  cuerpo_pb = crear_caja(
      "COMP_Portabaterias_2x18650_PROVISIONAL",
      (PARAMS["PORTABAT_ANCHO_X"], PARAMS["PORTABAT_LARGO_Y"], h_pb),
      (0.0, yb, z_pb + h_pb / 2.0),
      mat=MATS["CAUCHO"],
      col="04_Portabaterias_y_Bateria",
  )
  for offset_c in [-9.8, 9.8]:
    canal_celda = crear_caja(
        "Cut_Canal_Celda",
        (PARAMS["PORTABAT_ANCHO_X"] - 3.6, 19.0, h_pb),
        (0.0, yb + offset_c, z_pb + h_pb / 2.0 + 2.5),
    )
    booleana(cuerpo_pb, canal_celda, "DIFFERENCE")

  for sx in [-1.0, 1.0]:
    t_pb = crear_cilindro(
        "Cut_M3_PB", 1.8, 10.0, (sx * 22.0, yb, z_pb + 2.0), segs=20
    )
    booleana(cuerpo_pb, t_pb, "DIFFERENCE")
    av_pb = crear_cilindro(
        "Cut_Av_PB", 3.4, 4.0, (sx * 22.0, yb, z_pb + 3.5), segs=20
    )
    booleana(cuerpo_pb, av_pb, "DIFFERENCE")

  contactos = []
  for sx in [-1.0, 1.0]:
    for sy_c in [-9.8, 9.8]:
      cont = crear_caja(
          "Contacto_Resorte",
          (1.2, 8.0, 12.0),
          (sx * (PARAMS["PORTABAT_ANCHO_X"] / 2.0 - 2.5), yb + sy_c, z_pb + 10.0),
          mat=MATS["ACERO"],
      )
      contactos.append(cont)
  unir_objetos_en_uno(cuerpo_pb, contactos)
  mover_a_coleccion(cuerpo_pb, "04_Portabaterias_y_Bateria")

  for idx_c, sy_c in enumerate([-9.8, 9.8], start=1):
    celda = crear_cilindro(
        f"COMP_Celda_18650_Samsung_{idx_c}",
        9.0,
        65.0,
        (0.0, yb + sy_c, z_pb + 11.8),
        rot_deg=(0, 90, 0),
        segs=36,
        mat=MATS["CELDA_18650"],
        col="04_Portabaterias_y_Bateria",
    )
    polo_pos = crear_cilindro(
        f"Polo_Pos_{idx_c}",
        4.5,
        1.0,
        ((1 if idx_c == 1 else -1) * 32.6, yb + sy_c, z_pb + 11.8),
        rot_deg=(0, 90, 0),
        segs=24,
        mat=MATS["ACERO"],
    )
    unir_objetos_en_uno(celda, [polo_pos])
    mover_a_coleccion(celda, "04_Portabaterias_y_Bateria")

  z_eva_sup = (
      Z_piso
      + PARAMS["ESPESOR_PAD_EVA_INF"]
      + PARAMS["PORTABAT_ALTO_Z"]
      + holg_z
      + PARAMS["ESPESOR_PAD_EVA_SUP"] / 2.0
  )
  crear_caja(
      "COMP_Pad_EVA_Superior_Puente",
      (68.0, 14.0, PARAMS["ESPESOR_PAD_EVA_SUP"] - 2.0 * holg_z),
      (0.0, yb, z_eva_sup),
      mat=MATS["EVA"],
      col="04_Portabaterias_y_Bateria",
  )

  x_bms = PARAMS["X_BOLSILLO_BMS"]
  bms_pcb = crear_caja(
      "COMP_BMS_2S_10A_PROVISIONAL",
      (2.0, 35.0, 11.0),
      (x_bms + 0.5, yb, Z_piso + 6.5),
      mat=MATS["PCB_VERDE"],
      col="04_Portabaterias_y_Bateria",
  )
  chips_bms = []
  for sy_ic in [-10.0, 0.0, 10.0]:
    ic = crear_caja(
        "IC_BMS",
        (1.4, 5.0, 4.0),
        (x_bms - 1.0, yb + sy_ic, Z_piso + 6.5),
        mat=MATS["CHIP_IC"],
    )
    chips_bms.append(ic)
  unir_objetos_en_uno(bms_pcb, chips_bms)
  mover_a_coleccion(bms_pcb, "04_Portabaterias_y_Bateria")


# ==============================================================================
# 8. ELECTRONICA DETALLADA (COLECCION 03)
# ==============================================================================
def construir_electronica_detallada(coords_postes_pcb):
  Z_piso = PARAMS["DESPEJE_SUELO_Z"] + PARAMS["ESPESOR_SUELO"]
  yp = PARAMS["Y_CENTRO_PCB"]
  holg_z = PARAMS["HOLGURA_ASIENTO_Z"]
  z_pcb = (
      Z_piso + PARAMS["ALTO_POSTES_PCB"] + holg_z + PARAMS["PCB_ESPESOR"] / 2.0
  )
  z_sup_pcb = z_pcb + PARAMS["PCB_ESPESOR"] / 2.0

  # 8.1 PCB portadora + zocalos + borneras + taladros M3
  pcb = crear_caja(
      "COMP_Placa_Portadora_PCB",
      (PARAMS["PCB_ANCHO_X"], PARAMS["PCB_LARGO_Y"], PARAMS["PCB_ESPESOR"]),
      (0.0, yp, z_pcb),
      mat=MATS["PCB_VERDE"],
      col="03_Componentes_Electronicos",
  )
  elementos_pcb = []
  h_zoc = 5.2
  for sx_z in [-11.5, 11.5]:
    zoc = crear_caja(
        "Zocalo_ESP32",
        (2.4, 34.0, h_zoc),
        (sx_z, -51.0, z_sup_pcb + h_zoc / 2.0),
        mat=MATS["CHIP_IC"],
    )
    elementos_pcb.append(zoc)
  for sx_d in [-32.0, -17.5]:
    zoc_d = crear_caja(
        "Zocalo_TB6612",
        (2.4, 20.0, h_zoc),
        (sx_d, -45.0, z_sup_pcb + h_zoc / 2.0),
        mat=MATS["CHIP_IC"],
    )
    elementos_pcb.append(zoc_d)
  for bx, by in [(28.0, -24.0), (-28.0, -24.0), (0.0, -24.0)]:
    born = crear_caja(
        "Bornera_2P",
        (8.5, 7.0, 8.5),
        (bx, by, z_sup_pcb + 4.25),
        mat=MATS["PCB_AZUL"],
    )
    elementos_pcb.append(born)
  unir_objetos_en_uno(pcb, elementos_pcb)
  mover_a_coleccion(pcb, "03_Componentes_Electronicos")

  for px, py in coords_postes_pcb:
    t_pcb = crear_cilindro(
        "Cut_M3_PCB",
        PARAMS["DIAM_TALADRO_M3_LIBRE"] / 2.0,
        6.0,
        (px, py, z_pcb),
        segs=20,
    )
    booleana(pcb, t_pcb, "DIFFERENCE")

  # 8.2 ESP32 DevKit V1
  z_esp = z_sup_pcb + h_zoc + holg_z + 0.8
  esp_base = crear_caja(
      "COMP_ESP32_DevKit_V1",
      (26.0, 50.0, 1.6),
      (0.0, -53.0, z_esp),
      mat=MATS["PCB_NEGRA"],
      col="03_Componentes_Electronicos",
  )
  blindaje_rf = crear_caja(
      "ESP32_WROOM_Shield",
      (16.0, 18.0, 3.0),
      (0.0, -41.0, z_esp + 2.3),
      mat=MATS["ACERO"],
  )
  puerto_usb = crear_caja(
      "ESP32_Puerto_MicroUSB",
      (7.8, 6.5, 2.8),
      (0.0, -76.5, z_esp + 2.2),
      mat=MATS["ACERO"],
  )
  btn_en = crear_caja(
      "Btn_EN", (3.0, 4.0, 2.0), (-9.0, -73.0, z_esp + 1.8), mat=MATS["ACERO"]
  )
  btn_boot = crear_caja(
      "Btn_BOOT", (3.0, 4.0, 2.0), (9.0, -73.0, z_esp + 1.8), mat=MATS["ACERO"]
  )
  unir_objetos_en_uno(esp_base, [blindaje_rf, puerto_usb, btn_en, btn_boot])
  mover_a_coleccion(esp_base, "03_Componentes_Electronicos")

  # 8.3 Driver TB6612FNG
  z_drv = z_sup_pcb + h_zoc + holg_z + 0.8
  drv_base = crear_caja(
      "COMP_Driver_TB6612FNG",
      (18.5, 21.0, 1.6),
      (-24.75, -45.0, z_drv),
      mat=MATS["PCB_ROJA"],
      col="03_Componentes_Electronicos",
  )
  ic_tb = crear_caja(
      "IC_Toshiba_TB6612",
      (6.5, 8.0, 1.8),
      (-24.75, -45.0, z_drv + 1.7),
      mat=MATS["CHIP_IC"],
  )
  unir_objetos_en_uno(drv_base, [ic_tb])
  mover_a_coleccion(drv_base, "03_Componentes_Electronicos")

  # 8.4 Regulador LM2596
  z_buck = z_sup_pcb + 2.0 + holg_z + 0.8
  buck_pcb = crear_caja(
      "COMP_Regulador_Buck_LM2596_PROVISIONAL",
      (20.0, 39.0, 1.6),
      (24.5, -48.5, z_buck),
      mat=MATS["PCB_AZUL"],
      col="03_Componentes_Electronicos",
  )
  bobina = crear_cilindro(
      "Bobina_LM2596",
      6.0,
      4.8,
      (24.5, -48.5, z_buck + 3.2),
      segs=24,
      mat=MATS["COBRE_BOBINA"],
  )
  cap_in = crear_cilindro(
      "Cap_In_LM2596",
      4.0,
      7.5,
      (20.5, -34.0, z_buck + 4.55),
      segs=20,
      mat=MATS["ACERO"],
  )
  cap_out = crear_cilindro(
      "Cap_Out_LM2596",
      4.0,
      7.5,
      (20.5, -61.0, z_buck + 4.55),
      segs=20,
      mat=MATS["ACERO"],
  )
  trimpot = crear_caja(
      "Potenciometro_3296W",
      (4.8, 9.5, 7.5),
      (29.5, -48.5, z_buck + 4.55),
      mat=MATS["PCB_AZUL"],
  )
  torn_pot = crear_cilindro(
      "Tornillo_Trimpot",
      1.1,
      1.2,
      (29.5, -44.5, z_buck + 8.9),
      segs=12,
      mat=MATS["BRONCE"],
  )
  unir_objetos_en_uno(buck_pcb, [bobina, cap_in, cap_out, trimpot, torn_pot])
  mover_a_coleccion(buck_pcb, "03_Componentes_Electronicos")

  # 8.5 Condensador 470 uF
  crear_cilindro(
      "COMP_Capacitor_470uF_16V",
      4.0,
      10.0,
      (-24.75, -62.5, z_sup_pcb + holg_z + 5.0),
      segs=24,
      mat=MATS["CHIP_IC"],
      col="03_Componentes_Electronicos",
  )

  # 8.6 Jumper modo RF
  jmp_base = crear_caja(
      "COMP_Jumper_Modo_RF_GPIO4",
      (5.08, 2.54, 6.0),
      (-9.0, -24.0, z_sup_pcb + holg_z + 3.0),
      mat=MATS["SOPORTE"],
      col="03_Componentes_Electronicos",
  )
  pines_jmp = crear_caja(
      "Pines_Jumper",
      (3.5, 0.8, 7.5),
      (-9.0, -24.0, z_sup_pcb + holg_z + 3.75),
      mat=MATS["BRONCE"],
  )
  unir_objetos_en_uno(jmp_base, [pines_jmp])
  mover_a_coleccion(jmp_base, "03_Componentes_Electronicos")

  # 8.7 Interruptor rocker KCD1 (en X = 56.0, Y = -68.0 mm)
  Z_tapa_base = (
      PARAMS["DESPEJE_SUELO_Z"] + PARAMS["ALTO_PARED_CHASIS"] + holg_z
  )
  Z_tapa_sup = Z_tapa_base + PARAMS["ALTO_TAPA"]
  x_sw = PARAMS["X_INTERRUPTOR_KCD1"]
  y_sw = PARAMS["Y_INTERRUPTOR_KCD1"]
  sw_cuerpo = crear_caja(
      "COMP_Interruptor_Rocker_KCD1",
      (13.0, 19.0, 14.0),
      (x_sw, y_sw, Z_tapa_sup - 7.0),
      mat=MATS["CHIP_IC"],
      col="03_Componentes_Electronicos",
  )
  sw_marco = crear_caja(
      "Marco_KCD1",
      (15.0, 21.0, 2.0),
      (x_sw, y_sw, Z_tapa_sup + holg_z + 1.0),
      mat=MATS["CHIP_IC"],
  )
  sw_boton = crear_caja(
      "Actuador_Rocker_Rojo",
      (10.5, 15.0, 4.5),
      (x_sw, y_sw, Z_tapa_sup + holg_z + 3.0),
      rot_deg=(12, 0, 0),
      mat=MATS["SW_ROJO"],
  )
  terminales_sw = []
  for sy_t in [-6.0, 0.0, 6.0]:
    term = crear_caja(
        "Faston_Sw",
        (0.8, 4.8, 6.0),
        (x_sw, y_sw + sy_t, Z_tapa_sup - 17.0),
        mat=MATS["BRONCE"],
    )
    terminales_sw.append(term)
  unir_objetos_en_uno(sw_cuerpo, [sw_marco, sw_boton] + terminales_sw)
  mover_a_coleccion(sw_cuerpo, "03_Componentes_Electronicos")

  # 8.8 LED indicador y resistencia serie
  led_cuerpo = crear_cilindro(
      "COMP_LED_Indicador_5mm",
      2.5,
      6.0,
      (0.0, -55.0, 71.0),
      segs=24,
      mat=MATS["LED_ENCENDIDO"],
      col="03_Componentes_Electronicos",
  )
  bpy.ops.mesh.primitive_uv_sphere_add(
      segments=24, ring_count=16, radius=2.5, location=(0.0, -55.0, 74.0)
  )
  cupula_led = bpy.context.active_object
  cupula_led.data.materials.append(MATS["LED_ENCENDIDO"])
  portaled = crear_cilindro(
      "Portaled_Bisel", 4.1, 3.0, (0.0, -55.0, 70.5), segs=24, mat=MATS["CHIP_IC"]
  )
  unir_objetos_en_uno(led_cuerpo, [cupula_led, portaled])
  mover_a_coleccion(led_cuerpo, "03_Componentes_Electronicos")

  crear_cilindro(
      "COMP_Resistencia_220_Ohm_LED",
      1.2,
      6.5,
      (7.0, -50.0, 58.2),
      rot_deg=(0, 90, 0),
      segs=16,
      mat=MATS["SOPORTE"],
      col="03_Componentes_Electronicos",
  )

  # 8.9 Portafusible en bahia delantera derecha despejada (X = 64.0, Y = 44.0 mm)
  crear_cilindro(
      "COMP_Portafusible_5A_PROVISIONAL",
      5.5,
      28.0,
      (PARAMS["X_PORTAFUSIBLE"], PARAMS["Y_PORTAFUSIBLE"], Z_piso + 6.5),
      rot_deg=(90, 0, 0),
      segs=24,
      mat=MATS["CHIP_IC"],
      col="03_Componentes_Electronicos",
  )

  # 8.10 Motorreductores TT
  z_eje = PARAMS["Z_EJE_MOTORES"]
  y_eje = PARAMS["Y_EJE_MOTORES"]
  xm_abs = PARAMS["X_CENTRO_MOTOR"]
  for sx, lado in [(-1.0, "Izq"), (1.0, "Der")]:
    xm = sx * xm_abs
    caja_tt = crear_caja(
        f"COMP_Motor_Amarillo_TT_{lado}",
        (PARAMS["MOTOR_TT_ANCHO"], 37.0, PARAMS["MOTOR_TT_ALTO"]),
        (xm, y_eje - 3.0, z_eje),
        mat=MATS["MOTOR_AMARILLO"],
        col="03_Componentes_Electronicos",
    )
    cil_dc = crear_cilindro(
        f"Motor_DC_Can_{lado}",
        9.1,
        31.0,
        (xm, y_eje - 37.0, z_eje),
        rot_deg=(90, 0, 0),
        segs=28,
        mat=MATS["ACERO"],
    )
    eje_blanco = crear_cilindro(
        f"Eje_Blanco_TT_{lado}",
        2.7,
        44.0,
        (xm + sx * 14.5, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=20,
        mat=MATS["EJE_BLANCO"],
    )
    leng_1 = crear_caja(
        f"Tab1_{lado}",
        (2.6, 3.0, 0.6),
        (xm - 4.0, y_eje - 53.0, z_eje),
        mat=MATS["BRONCE"],
    )
    leng_2 = crear_caja(
        f"Tab2_{lado}",
        (2.6, 3.0, 0.6),
        (xm + 4.0, y_eje - 53.0, z_eje),
        mat=MATS["BRONCE"],
    )
    unir_objetos_en_uno(caja_tt, [cil_dc, eje_blanco, leng_1, leng_2])
    mover_a_coleccion(caja_tt, "03_Componentes_Electronicos")


# ==============================================================================
# 9. CABLEADO (COLECCION 06)
# ==============================================================================
def crear_cable_con_terminales(nombre, puntos_xyz, radio, mat_cable):
  curva = bpy.data.curves.new(name=nombre, type="CURVE")
  curva.dimensions = "3D"
  curva.bevel_depth = radio
  curva.bevel_resolution = 6

  spline = curva.splines.new("NURBS")
  spline.points.add(len(puntos_xyz) - 1)
  for i, pt in enumerate(puntos_xyz):
    spline.points[i].co = (pt[0], pt[1], pt[2], 1.0)
  spline.use_endpoint_u = True
  spline.order_u = min(3, len(puntos_xyz))

  obj_c = bpy.data.objects.new(nombre, curva)
  obj_c.data.materials.append(mat_cable)
  COLS["06_Cableado"].objects.link(obj_c)
  return obj_c


def construir_cableado_completo():
  crear_cable_con_terminales(
      "CAB_01_Potencia_BMS_a_Interruptor",
      [
          (44.2, 5.5, 35.0),
          (44.2, -5.0, 38.5),
          (44.2, -61.5, 38.5),
          (48.0, -61.5, 40.5),
          (52.0, -61.5, 42.0),
      ],
      1.0,
      MATS["CABLE_POS"],
  )
  crear_cable_con_terminales(
      "CAB_02_Interruptor_a_Bornera_PCB",
      [
          (52.0, -68.0, 39.5),
          (39.5, -60.0, 49.5),
          (38.5, -16.5, 51.0),
          (28.0, -18.5, 51.0),
      ],
      1.0,
      MATS["CABLE_POS"],
  )
  crear_cable_con_terminales(
      "CAB_03_Switch_a_LED_Obligatorio",
      [
          (52.0, -74.0, 39.5),
          (38.0, -76.5, 58.0),
          (12.0, -66.0, 58.0),
          (1.8, -55.0, 58.0),
          (1.8, -55.0, 59.5),
          (1.8, -55.0, 66.5),
      ],
      0.9,
      MATS["LED_ENCENDIDO"],
  )
  crear_cable_con_terminales(
      "CAB_04_Retorno_GND_LED",
      [
          (-1.8, -55.0, 66.5),
          (-1.8, -55.0, 59.5),
          (-1.8, -55.0, 58.0),
          (-6.0, -50.0, 58.0),
          (-14.5, -36.0, 58.0),
          (-14.5, -16.5, 51.0),
          (-2.0, -18.5, 51.0),
      ],
      0.9,
      MATS["CABLE_GND"],
  )
  crear_cable_con_terminales(
      "CAB_05_Masa_Principal_Bateria_GND",
      [
          (41.0, 1.0, 31.0),
          (19.0, -1.5, 36.0),
          (0.0, -12.0, 42.0),
          (2.0, -18.5, 50.5),
      ],
      1.0,
      MATS["CABLE_GND"],
  )
  crear_cable_con_terminales(
      "CAB_06_Potencia_Motor_Izquierdo",
      [
          (-28.0, -18.5, 51.0),
          (-20.0, -16.5, 51.0),
          (-20.0, -16.5, 38.0),
          (-20.0, -76.5, 38.0),
          (-26.5, -76.5, 32.5),
      ],
      1.0,
      MATS["CABLE_MOT"],
  )
  crear_cable_con_terminales(
      "CAB_07_Potencia_Motor_Derecho",
      [
          (25.0, -18.5, 51.0),
          (20.0, -16.5, 51.0),
          (20.0, -16.5, 38.0),
          (20.0, -76.5, 38.0),
          (26.5, -76.5, 32.5),
      ],
      1.0,
      MATS["CABLE_MOT"],
  )
  crear_cable_con_terminales(
      "CAB_08_Alimentacion_5V_LM2596_a_ESP32",
      [(20.5, -69.5, 52.5), (15.5, -75.0, 56.5), (11.5, -75.0, 56.5)],
      0.9,
      MATS["CABLE_PWM"],
  )


# ==============================================================================
# 10. AUXILIARES: RUEDAS, BALL CASTER, BALON, GALGA Y CAMARAS (COLECCION 07)
# ==============================================================================
def construir_auxiliares():
  L = PARAMS["LARGO_CHASIS"]
  y_eje = PARAMS["Y_EJE_MOTORES"]
  z_eje = PARAMS["Z_EJE_MOTORES"]
  xr_abs = PARAMS["X_CENTRO_RUEDA"]
  holg_eje = PARAMS["HOLGURA_EJE_PASO"]

  # 10.1 Ruedas (neumatico anular + rin con cubo para eje y cajeado para tornillo M2.5)
  for sx, lado in [(-1.0, "Izq"), (1.0, "Der")]:
    xr = sx * xr_abs
    caucho = crear_cilindro(
        f"AUX_Rueda_65mm_Caucho_{lado}",
        PARAMS["RUEDA_DIAM"] / 2.0,
        PARAMS["RUEDA_ANCHO"],
        (xr, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=64,
        mat=MATS["CAUCHO"],
        col="07_Elementos_Auxiliares",
    )
    hueco_rin_en_caucho = crear_cilindro(
        f"Cut_Alojamiento_Rin_{lado}",
        22.0 + 0.2,
        PARAMS["RUEDA_ANCHO"] + 2.0,
        (xr, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=36,
    )
    booleana(caucho, hueco_rin_en_caucho, "DIFFERENCE")

    rin = crear_cilindro(
        f"AUX_Rueda_Rin_Amarillo_{lado}",
        22.0,
        PARAMS["RUEDA_ANCHO"] + 0.4,
        (xr, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=36,
        mat=MATS["MOTOR_AMARILLO"],
        col="07_Elementos_Auxiliares",
    )
    hueco_eje_rin = crear_cilindro(
        f"Cut_Eje_Rin_{lado}",
        2.7 + holg_eje,
        PARAMS["RUEDA_ANCHO"] + 4.0,
        (xr, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=24,
    )
    booleana(rin, hueco_eje_rin, "DIFFERENCE")
    cajeado_torn_rin = crear_cilindro(
        f"Cut_Cajeado_Tornillo_Rin_{lado}",
        4.2,
        6.0,
        (xr + sx * 11.5, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=24,
    )
    booleana(rin, cajeado_torn_rin, "DIFFERENCE")

    crear_cilindro(
        f"AUX_Tornillo_Eje_M2_5_{lado}",
        2.2,
        3.6,
        (xr + sx * 11.2, y_eje, z_eje),
        rot_deg=(0, 90, 0),
        segs=16,
        mat=MATS["ACERO"],
        col="07_Elementos_Auxiliares",
    )

  # 10.2 Ball caster con cavidad interior para la esfera de Ø15 mm
  yc = PARAMS["Y_BALL_CASTER"]
  cuerpo_caster = crear_cilindro(
      "AUX_BallCaster_Cuerpo",
      11.0,
      5.8,
      (0.0, yc, 6.9),
      segs=32,
      mat=MATS["ACERO"],
      col="07_Elementos_Auxiliares",
  )
  hueco_bola_caster = crear_cilindro(
      "Cut_Cavidad_Bola_Caster", 7.8, 8.0, (0.0, yc, 6.9), segs=32
  )
  booleana(cuerpo_caster, hueco_bola_caster, "DIFFERENCE")

  bpy.ops.mesh.primitive_uv_sphere_add(
      segments=32, ring_count=16, radius=7.5, location=(0.0, yc, 7.5)
  )
  esfera_c = bpy.context.active_object
  esfera_c.name = "AUX_BallCaster_Esfera_15mm"
  esfera_c.data.materials.append(MATS["ACERO"])
  mover_a_coleccion(esfera_c, "07_Elementos_Auxiliares")

  # 10.3 Balon reglamentario
  r_balon = PARAMS["DIAM_BALON"] / 2.0
  y_balon = (L / 2.0) - PARAMS["PROF_CAVIDAD_DISENO"] + r_balon + 0.3
  bpy.ops.mesh.primitive_uv_sphere_add(
      segments=48, ring_count=24, radius=r_balon, location=(0.0, y_balon, r_balon)
  )
  balon = bpy.context.active_object
  balon.name = "AUX_Balon_Reglamentario_35_5mm"
  balon.data.materials.append(MATS["BALON"])
  mover_a_coleccion(balon, "07_Elementos_Auxiliares")

  # 10.4 Galga del limite legal
  y_limite = (L / 2.0) - PARAMS["LIMITE_CAVIDAD_MM"]
  crear_caja(
      "AUX_Galga_Limite_10_65mm",
      (80.0, 0.4, 48.0),
      (0.0, y_limite, 26.0),
      mat=MATS["GALGA_LIMITE"],
      col="07_Elementos_Auxiliares",
  )

  # 10.5 Leyenda 3D flotante
  bpy.ops.object.text_add(
      location=(-95.0, 100.0, 5.0), rotation=(math.radians(90), 0.0, 0.0)
  )
  txt = bpy.context.active_object
  txt.name = "AUX_Leyenda_3D"
  txt.data.body = (
      "ROBOGOL SD2026-2 - USCO (CORREGIDO V4)\n"
      "Azul: P01 Chasis | Gris: P02 Tapa\n"
      "Naranja: P03-P06 Soportes\n"
      "Rojo/Negro: cables de potencia | Azul: 5V"
  )
  txt.data.size = 5.0
  txt.data.materials.append(MATS["TEXTO_LEYENDA"])
  mover_a_coleccion(txt, "07_Elementos_Auxiliares")

  # 10.6 Luz
  luz_data = bpy.data.lights.new(name="LUZ_Sol", type="SUN")
  luz_data.energy = 3.0
  luz = bpy.data.objects.new("LUZ_Sol", luz_data)
  luz.location = (200.0, -200.0, 400.0)
  luz.rotation_euler = (math.radians(40), 0.0, math.radians(30))
  COLS["07_Elementos_Auxiliares"].objects.link(luz)

  # 10.7 Camaras de las 6 vistas
  definicion_vistas = [
      (
          "CAM_V1_Exterior_Isometrica",
          (240.0, 240.0, 180.0),
          (math.radians(60), 0.0, math.radians(135)),
      ),
      (
          "CAM_V2_Interior_Sin_Tapa",
          (180.0, -210.0, 195.0),
          (math.radians(48), 0.0, math.radians(40)),
      ),
      (
          "CAM_V3_Vista_Explotada",
          (290.0, -240.0, 210.0),
          (math.radians(58), 0.0, math.radians(50)),
      ),
      ("CAM_V4_Superior_Planta_XY", (0.0, 0.0, 380.0), (0.0, 0.0, 0.0)),
      (
          "CAM_V5_Soportes_Fijaciones",
          (0.0, -230.0, 165.0),
          (math.radians(55), 0.0, 0.0),
      ),
      (
          "CAM_V6_Rutas_Cableado",
          (130.0, -140.0, 210.0),
          (math.radians(40), 0.0, math.radians(42)),
      ),
  ]
  camaras = []
  for nombre_cam, pos_cam, rot_cam in definicion_vistas:
    cam_data = bpy.data.cameras.new(name=nombre_cam)
    cam_data.clip_end = 2000.0
    if "V4" in nombre_cam:
      cam_data.type = "ORTHO"
      cam_data.ortho_scale = 260.0
    cam_obj = bpy.data.objects.new(nombre_cam, cam_data)
    cam_obj.location = pos_cam
    cam_obj.rotation_euler = rot_cam
    COLS["07_Elementos_Auxiliares"].objects.link(cam_obj)
    camaras.append(cam_obj)
  return camaras


# ==============================================================================
# 11. CONFIGURACION DE LAS 6 VISTAS EN LA LINEA DE TIEMPO (FRAMES 1-6)
# ==============================================================================
def configurar_vistas(camaras):
  scn = bpy.context.scene
  nombres = [
      "V1 Exterior Ensamblado",
      "V2 Interior sin Tapa",
      "V3 Explosionada",
      "V4 Planta Superior XY",
      "V5 Soportes y Fijaciones M3",
      "V6 Conexiones y Cableado",
  ]
  for i, cam in enumerate(camaras, start=1):
    m = scn.timeline_markers.new(nombres[i - 1], frame=i)
    m.camera = cam
  scn.camera = camaras[0]

  offsets = {
      "02_Tapa": Vector((0, 0, 90)),
      "03_Componentes_Electronicos": Vector((0, 0, 45)),
      "04_Portabaterias_y_Bateria": Vector((0, 0, 25)),
      "05_Soportes_y_Fijaciones": Vector((0, 0, 65)),
      "06_Cableado": Vector((0, 0, 45)),
  }
  for nombre_col, off in offsets.items():
    for obj in COLS[nombre_col].objects:
      base = obj.location.copy()
      for f in range(1, 7):
        obj.location = base + (off if f == 3 else Vector((0, 0, 0)))
        obj.keyframe_insert(data_path="location", frame=f)
      obj.location = base

  for obj in COLS["02_Tapa"].objects:
    for f in range(1, 7):
      oculto = f in (2, 4, 5, 6)
      obj.hide_viewport = oculto
      obj.hide_render = oculto
      obj.keyframe_insert(data_path="hide_viewport", frame=f)
      obj.keyframe_insert(data_path="hide_render", frame=f)

  scn.frame_set(1)

  if PARAMS["RENDERIZAR_PNG_VISTAS"]:
    scn.render.resolution_x = 1600
    scn.render.resolution_y = 1200
    for i in range(1, 7):
      scn.frame_set(i)
      scn.render.filepath = os.path.join(OUTPUT_DIR, f"Vista_{i}.png")
      bpy.ops.render.render(write_still=True)
    scn.frame_set(1)


# ==============================================================================
# 12. COMPROBACION AUTOMATICA DE COLISIONES Y AUDITORIA GEOMETRICA
# ==============================================================================
def volumen_mm3(obj):
  bm = bmesh.new()
  bm.from_mesh(obj.data)
  bmesh.ops.transform(bm, matrix=obj.matrix_world, verts=bm.verts)
  vol = abs(bm.calc_volume())
  bm.free()
  return vol


def bbox_mm(obj):
  esquinas = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
  mn = Vector((
      min(c.x for c in esquinas),
      min(c.y for c in esquinas),
      min(c.z for c in esquinas),
  ))
  mx = Vector((
      max(c.x for c in esquinas),
      max(c.y for c in esquinas),
      max(c.z for c in esquinas),
  ))
  return mn, mx


def aabb_se_solapan(mn1, mx1, mn2, mx2, margen=0.0):
  return (
      (mn1.x - margen <= mx2.x and mx1.x + margen >= mn2.x)
      and (mn1.y - margen <= mx2.y and mx1.y + margen >= mn2.y)
      and (mn1.z - margen <= mx2.z and mx1.z + margen >= mn2.z)
  )


def es_interseccion_intencional(nombre_a, nombre_b):
  """
    Filtra uniones intencionales de ensamblaje cuando existan (por ejemplo,
    la galga fantasma de referencia reglamentaria).
    """
  par = {nombre_a, nombre_b}
  if "AUX_Galga_Limite_10_65mm" in par:
    return True
  return False


def construir_bmesh_evaluado_mundo(obj, depsgraph):
  """
    Obtiene la malla final evaluada en espacio mundial (incluyendo curvas NURBS
    con bevel y modificadores aplicados) mediante el grafo de dependencias.
    """
  obj_eval = obj.evaluated_get(depsgraph)
  mesh_eval = obj_eval.to_mesh()
  bm = bmesh.new()
  bm.from_mesh(mesh_eval)
  bmesh.ops.transform(bm, matrix=obj_eval.matrix_world, verts=bm.verts)
  bmesh.ops.triangulate(bm, faces=bm.faces)
  obj_eval.to_mesh_clear()
  return bm


def volumen_interseccion_booleana_mm3(obj_a, obj_b, depsgraph):
  """
    Calcula el volumen real de interpenetracion (mm3) entre dos objetos usando
    un modificador Boolean EXACT temporal sobre copias evaluadas, evitando falsos
    positivos por simple roce superficial o cajas delimitadoras superpuestas.
    """
  bm_a = construir_bmesh_evaluado_mundo(obj_a, depsgraph)
  bm_b = construir_bmesh_evaluado_mundo(obj_b, depsgraph)

  mesh_a = bpy.data.meshes.new("_tmp_col_a")
  mesh_b = bpy.data.meshes.new("_tmp_col_b")
  bm_a.to_mesh(mesh_a)
  bm_b.to_mesh(mesh_b)
  bm_a.free()
  bm_b.free()

  tmp_a = bpy.data.objects.new("_tmp_col_a", mesh_a)
  tmp_b = bpy.data.objects.new("_tmp_col_b", mesh_b)
  bpy.context.scene.collection.objects.link(tmp_a)
  bpy.context.scene.collection.objects.link(tmp_b)

  mod = tmp_a.modifiers.new(name="Bool_Intersect", type="BOOLEAN")
  mod.operation = "INTERSECT"
  mod.solver = "EXACT"
  mod.object = tmp_b

  bpy.context.view_layer.update()
  dg = bpy.context.evaluated_depsgraph_get()
  eval_a = tmp_a.evaluated_get(dg)
  m_int = eval_a.to_mesh()
  bm_int = bmesh.new()
  bm_int.from_mesh(m_int)
  vol = abs(bm_int.calc_volume()) if len(bm_int.faces) > 0 else 0.0
  bm_int.free()
  eval_a.to_mesh_clear()

  bpy.data.objects.remove(tmp_a, do_unlink=True)
  bpy.data.objects.remove(tmp_b, do_unlink=True)
  bpy.data.meshes.remove(mesh_a)
  bpy.data.meshes.remove(mesh_b)
  return vol


def comprobar_colisiones_ensamblaje():
  """
    Ejecuta la verificacion automatica de colisiones en 3 etapas:
      1. Fase amplia (Broad-phase): superposicion de cajas delimitadoras (AABB).
         No se asume como colision real para evitar falsos positivos.
      2. Fase estrecha (Narrow-phase): interseccion exacta de triangulos con BVHTree
         sobre la geometria evaluada en el depsgraph.
      3. Confirmacion volumetrica: calculo de volumen booleano EXACT (mm3).
    """
  bpy.context.scene.frame_set(1)
  bpy.context.view_layer.update()
  depsgraph = bpy.context.evaluated_depsgraph_get()

  objetos_validos = [
      obj
      for obj in bpy.context.scene.objects
      if obj.type in ("MESH", "CURVE")
      and not es_interseccion_intencional(obj.name, obj.name)
  ]

  datos_obj = {}
  for obj in objetos_validos:
    mn, mx = bbox_mm(obj)
    bm = construir_bmesh_evaluado_mundo(obj, depsgraph)
    bvh = BVHTree.FromBMesh(bm, epsilon=0.0005)
    datos_obj[obj.name] = (obj, mn, mx, bm, bvh)

  pares_aabb = 0
  colisiones_confirmadas = []

  nombres = sorted(datos_obj.keys())
  for i in range(len(nombres)):
    na = nombres[i]
    obj_a, mn_a, mx_a, bm_a, bvh_a = datos_obj[na]
    for j in range(i + 1, len(nombres)):
      nb = nombres[j]
      if es_interseccion_intencional(na, nb):
        continue
      obj_b, mn_b, mx_b, bm_b, bvh_b = datos_obj[nb]

      # Etapa 1: filtro rapido por AABB
      if not aabb_se_solapan(mn_a, mx_a, mn_b, mx_b, margen=0.0):
        continue
      pares_aabb += 1

      # Etapa 2: prueba de interseccion exacta de mallas mediante BVHTree
      solapes_bvh = bvh_a.overlap(bvh_b)
      if not solapes_bvh:
        continue

      # Etapa 3: confirmacion de volumen real de interseccion booleana
      vol_int = volumen_interseccion_booleana_mm3(obj_a, obj_b, depsgraph)
      if vol_int > 0.05:
        colisiones_confirmadas.append((na, nb, len(solapes_bvh), vol_int))

  for _, _, _, bm, _ in datos_obj.values():
    bm.free()

  return len(objetos_validos), pares_aabb, colisiones_confirmadas


def auditar_y_reportar(piezas_stl):
  dens = PARAMS["DENSIDAD_PLA_G_CM3"]
  fact = PARAMS["FACTOR_MASA_EFECTIVA_3D"]
  lineas = [
      "# Reporte de Auditoria CAD/DFM/DFA - RoboGol SD2026-2 (V4 Corregido)",
      "",
      "## 1. Piezas imprimibles en 3D (FDM PLA)",
      "",
      "| Pieza | Volumen (cm3) | Masa solida (g) | Masa efectiva FDM (g) |",
      "|---|---|---|---|",
  ]
  masa_imp = 0.0
  for p in piezas_stl:
    v_cm3 = volumen_mm3(p) / 1000.0
    m_sol = v_cm3 * dens
    m_ef = m_sol * fact
    masa_imp += m_ef
    lineas.append(f"| {p.name} | {v_cm3:.2f} | {m_sol:.1f} | {m_ef:.1f} |")

  comerciales = {
      "2 celdas 18650": 90.0,
      "2 motores TT": 60.0,
      "2 ruedas 65 mm": 60.0,
      "ESP32 + driver + LM2596 + PCB": 45.0,
      "Portabaterias + BMS": 25.0,
      "Cableado, interruptor, LED, fusible": 25.0,
      "Tornilleria M3": 20.0,
      "Ball caster": 10.0,
  }
  masa_com = sum(comerciales.values())
  total = masa_imp + masa_com
  lineas += [
      "",
      f"- Masa impresa efectiva: **{masa_imp:.1f} g**",
      f"- Masa comercial estimada: **{masa_com:.1f} g**",
      (
          f"- **Masa total estimada: {total:.1f} g** (limite"
          f" {PARAMS['LIMITE_MASA_MAX_G']} g, meta"
          f" {PARAMS['OBJETIVO_MASA_ING_G']} g)"
      ),
      (
          "- Cumple limite de masa:"
          f" **{'SI' if total <= PARAMS['LIMITE_MASA_MAX_G'] else 'NO'}**"
      ),
      "",
  ]

  # Dimensiones globales del robot (excluyendo el balon externo y la galga)
  mn_g = Vector((1e9, 1e9, 1e9))
  mx_g = Vector((-1e9, -1e9, -1e9))
  for obj in bpy.context.scene.objects:
    if obj.type != "MESH" or "Balon" in obj.name or "Galga" in obj.name:
      continue
    a, b = bbox_mm(obj)
    mn_g = Vector((min(mn_g.x, a.x), min(mn_g.y, a.y), min(mn_g.z, a.z)))
    mx_g = Vector((max(mx_g.x, b.x), max(mx_g.y, b.y), max(mx_g.z, b.z)))
  dim = mx_g - mn_g
  ok_dim = max(dim.x, dim.y, dim.z) <= PARAMS["LIMITE_DIM_MAX"]
  lineas += [
      "## 2. Dimensiones globales y despejes",
      "",
      f"- Envolvente del robot: **{dim.x:.1f} x {dim.y:.1f} x {dim.z:.1f} mm** "
      f"(limite {PARAMS['LIMITE_DIM_MAX']} mm):"
      f" **{'CUMPLE' if ok_dim else 'NO CUMPLE'}**",
      f"- Despeje libre al suelo del chasis: **{PARAMS['DESPEJE_SUELO_Z']:.1f}"
      " mm**",
      (
          "- Profundidad de cavidad frontal:"
          f" **{PARAMS['PROF_CAVIDAD_DISENO']:.2f} mm** (limite"
          f" {PARAMS['LIMITE_CAVIDAD_MM']} mm):"
          f" **{'CUMPLE' if PARAMS['PROF_CAVIDAD_DISENO'] <= PARAMS['LIMITE_CAVIDAD_MM'] else 'NO CUMPLE'}**"
      ),
      "",
  ]

  # Comprobacion automatica de colisiones
  n_objs, n_aabb, colisiones = comprobar_colisiones_ensamblaje()
  lineas += [
      (
          "## 3. Comprobacion automatica de colisiones (Depsgraph + AABB +"
          " BVHTree + Boolean Exact)"
      ),
      "",
      f"- Objetos evaluados en escena: **{n_objs}**",
      (
          "- Pares con cajas delimitadoras (AABB) proximas o superpuestas:"
          f" **{n_aabb}**"
      ),
      f"- **Colisiones fisicas reales confirmadas: {len(colisiones)}**",
      "",
  ]
  if colisiones:
    lineas += [
        (
            "| Objeto A | Objeto B | Triangulos en contacto | Volumen"
            " interseccion (mm3) |"
        ),
        "|---|---|---|---|",
    ]
    for na, nb, n_tri, vol_c in colisiones:
      lineas.append(f"| {na} | {nb} | {n_tri} | {vol_c:.2f} |")
  else:
    lineas.append(
        "- Resultado: **0 intersecciones no deseadas detectadas en todo el"
        " ensamblaje.**"
    )

  ruta_md = os.path.join(OUTPUT_DIR, "Reporte_Auditoria_RoboGol.md")
  with open(ruta_md, "w", encoding="utf-8") as f:
    f.write("\n".join(lineas))
  print("\n".join(lineas))


# ==============================================================================
# 13. EJECUCION PRINCIPAL Y EXPORTACION
# ==============================================================================
chasis_obj, c_postes, c_columnas, c_suelo = construir_p01_chasis()
tapa_obj = construir_p02_tapa(c_columnas)
soportes_objs = construir_soportes_y_fijaciones(c_postes, c_columnas, c_suelo)
construir_sistema_bateria_completo()
construir_electronica_detallada(c_postes)
construir_cableado_completo()
camaras_vistas = construir_auxiliares()
configurar_vistas(camaras_vistas)

piezas_stl = [chasis_obj, tapa_obj] + soportes_objs
auditar_y_reportar(piezas_stl)

for pieza in piezas_stl:
  bpy.ops.object.select_all(action="DESELECT")
  pieza.select_set(True)
  bpy.context.view_layer.objects.active = pieza
  ruta_stl = os.path.join(OUTPUT_DIR, f"{pieza.name}.stl")
  if hasattr(bpy.ops.wm, "stl_export"):
    bpy.ops.wm.stl_export(filepath=ruta_stl, export_selected_objects=True)
  else:
    bpy.ops.export_mesh.stl(filepath=ruta_stl, use_selection=True)

ruta_blend = os.path.join(OUTPUT_DIR, "RoboGol_SD20262_Ensamblaje_V2.blend")
bpy.ops.wm.save_as_mainfile(filepath=ruta_blend)
print(f"=== EXITO TOTAL: Archivos guardados en: {OUTPUT_DIR} ===")