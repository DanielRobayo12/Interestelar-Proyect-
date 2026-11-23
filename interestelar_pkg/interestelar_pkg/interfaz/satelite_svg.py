"""Dibujo SVG de un satelite generico (para mostrar en la interfaz).

Uso en PySide6:
    from PySide6.QtSvgWidgets import QSvgWidget
    from PySide6.QtCore import QByteArray
    from .satelite_svg import SVG_SATELITE
    w = QSvgWidget(); w.load(QByteArray(SVG_SATELITE.encode("utf-8")))
"""

SVG_SATELITE = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 400 260" width="400" height="260">
  <defs>
    <linearGradient id="panel" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#2a63c4"/>
      <stop offset="1" stop-color="#0d2c6b"/>
    </linearGradient>
    <linearGradient id="body" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#ecc968"/>
      <stop offset="1" stop-color="#b8892e"/>
    </linearGradient>
    <linearGradient id="dish" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#f1f4f8"/>
      <stop offset="1" stop-color="#a9b4c2"/>
    </linearGradient>
    <pattern id="grid" width="20" height="17.5" patternUnits="userSpaceOnUse">
      <path d="M20 0H0V17.5" fill="none" stroke="#7fa9ee" stroke-width="1"/>
    </pattern>
  </defs>

  <!-- orbita decorativa -->
  <ellipse cx="200" cy="135" rx="192" ry="58" fill="none" stroke="#3b4a6b" stroke-width="1.5" stroke-dasharray="6 6"/>

  <!-- estrellas -->
  <g fill="#cfd8ee">
    <circle cx="40" cy="40" r="1.6"/><circle cx="95" cy="22" r="1.2"/><circle cx="330" cy="35" r="1.6"/>
    <circle cx="372" cy="80" r="1.2"/><circle cx="60" cy="225" r="1.4"/><circle cx="330" cy="228" r="1.6"/>
    <circle cx="150" cy="238" r="1.1"/><circle cx="268" cy="22" r="1.1"/>
  </g>

  <!-- brazos de los paneles -->
  <g stroke="#8d96a3" stroke-width="5" stroke-linecap="round">
    <line x1="140" y1="130" x2="168" y2="130"/>
    <line x1="232" y1="130" x2="260" y2="130"/>
  </g>

  <!-- panel solar izquierdo -->
  <g transform="translate(18,95)">
    <rect width="122" height="70" fill="url(#panel)" stroke="#a9c1ee" stroke-width="2"/>
    <rect width="122" height="70" fill="url(#grid)"/>
    <line x1="61" y1="0" x2="61" y2="70" stroke="#a9c1ee" stroke-width="2"/>
  </g>
  <!-- panel solar derecho -->
  <g transform="translate(260,95)">
    <rect width="122" height="70" fill="url(#panel)" stroke="#a9c1ee" stroke-width="2"/>
    <rect width="122" height="70" fill="url(#grid)"/>
    <line x1="61" y1="0" x2="61" y2="70" stroke="#a9c1ee" stroke-width="2"/>
  </g>

  <!-- antena parabolica -->
  <line x1="200" y1="96" x2="200" y2="72" stroke="#8d96a3" stroke-width="5" stroke-linecap="round"/>
  <path d="M170 50 Q200 92 230 50 Z" fill="url(#dish)" stroke="#7b8594" stroke-width="2" stroke-linejoin="round"/>
  <ellipse cx="200" cy="50" rx="30" ry="6" fill="#d8dee7" stroke="#7b8594" stroke-width="2"/>
  <line x1="200" y1="52" x2="200" y2="30" stroke="#7b8594" stroke-width="3" stroke-linecap="round"/>
  <circle cx="200" cy="28" r="4" fill="#e5484d"/>

  <!-- cuerpo -->
  <rect x="166" y="95" width="68" height="70" rx="7" fill="url(#body)" stroke="#8a6620" stroke-width="2"/>
  <line x1="166" y1="112" x2="234" y2="112" stroke="#8a6620" stroke-width="1.5" opacity="0.7"/>
  <line x1="166" y1="148" x2="234" y2="148" stroke="#8a6620" stroke-width="1.5" opacity="0.7"/>

  <!-- sensor / lente -->
  <circle cx="200" cy="130" r="15" fill="#171c28" stroke="#5b6475" stroke-width="2"/>
  <circle cx="200" cy="130" r="8" fill="#3b82f6" opacity="0.85"/>
  <circle cx="196" cy="126" r="2.5" fill="#dbeafe"/>

  <!-- antenita lateral -->
  <line x1="234" y1="104" x2="250" y2="86" stroke="#8d96a3" stroke-width="2.5" stroke-linecap="round"/>
  <circle cx="251" cy="85" r="2.5" fill="#8d96a3"/>
</svg>
"""
