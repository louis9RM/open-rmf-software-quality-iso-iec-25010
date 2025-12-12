**1\. Métricas Principales por Característica de Calidad ISO 25010**

***Contexto***
Para el análisis de característica de calidad, usando la ISO 25010, se consideran 2 componentes importantes [rmf-core](https://github.com/open-rmf/rmf_demos.git) y [rmf-web](https://github.com/open-rmf/rmf-web.git) del proyecto [Open-RMF](https://www.open-rmf.org/). Los lenguajes de programación usados el los componentes son:
* rmf-core: C++
* rmf_web: TypeScript / JavaScript / Python

El modelo ISO/IEC 25010 define las características de calidad del producto software, tales como Adecuación Funcional, Eficiencia de Desempeño,
 Compatibilidad, Capacidad de Interacción, Fiabilidad, Seguridad, Mantenibilidad, Flexibilidad y Protección. Ahora bien, aplicando el modelo ISO 25010 a nuestro contexto nos permite identificar métricas relevantes para cada una de las características mencionadas, aplicables a los lenguajes de programación indicados: C++, Python, TypeScript, JavaScript.

| Característica de Calidad | Métrica Principal (Ejemplos) | Subcaracterística Relacionada |
| :---- | :---- | :---- |
| **Adecuación Funcional** | **Porcentaje de Cobertura de Requisitos Funcionales** ($\\%\\text{CRF}$) | Completitud Funcional |
|  | **Tasa de Errores Funcionales por Módulo/Prueba** ($\\frac{\\text{No. Errores}}{\\text{No. Módulos/Pruebas}}$) | Corrección Funcional |
| **Eficiencia de Desempeño** | **Tiempo de Respuesta Promedio** ($\\text{TR}$ en ms) | Comportamiento Temporal |
|  | **Utilización de CPU/Memoria bajo Carga** ($\\text{Recursos}$ en $\\%$ o MB) | Utilización de Recursos |
|  | **Tasa de Transacciones por Segundo** ($\\text{TPS}$) o **Latencia Promedio** ($\\text{L}$ en ms) | Capacidad |
| **Compatibilidad** | **Porcentaje de Interoperabilidad Exitosa** ($\\%\\text{IE}$) | Interoperabilidad |
|  | **Porcentaje de Recursos Compartidos sin Conflicto** ($\\%\\text{RSC}$) | Coexistencia |
| **Capacidad de Interacción** (Usabilidad) | **Tasa de Éxito de Tareas** ($\\%\\text{ET}$) / **Tiempo para Realizar Tarea** ($\\text{TTRT}$) | Operabilidad / Aprendizabilidad |
|  | **Nivel de Satisfacción del Usuario** (p.e., Net Promoter Score, $\\text{CSAT}$) | Satisfacción del Usuario |
| **Fiabilidad** | **Tiempo Medio Entre Fallos** ($\\text{MTBF}$ en horas) | Madurez |
|  | **Tiempo Medio de Recuperación** ($\\text{MTTR}$ en minutos) | Capacidad de Recuperación |
|  | **Densidad de Defectos por Mil Líneas de Código** ($\\text{DLOC}$ por KLOC) | Tolerancia a Fallos |
| **Seguridad** | **Tasa de Vulnerabilidades de Gravedad Alta y Media** ($\\text{VGA}$ / $\\text{VGM}$) | Confidencialidad / Integridad |
|  | **Porcentaje de Cobertura de Pruebas de Seguridad** ($\\%\\text{CPS}$) | Resistencia a Ataques |
| **Mantenibilidad** | **Complejidad Ciclomática Promedio por Módulo** ($\\text{CC}$ promedio) | Analizabilidad |
|  | **Índice de Mantenibilidad** ($\\text{IM}$ / $\\text{Maintainability Index}$) | Capacidad de Modificación |
|  | **Acoplamiento / Cohesión** (p.e., $\\text{CBO}$ / $\\text{LCOM}$) | Modularidad |
| **Flexibilidad** (Portabilidad) | **Tiempo de Adaptación a Nuevo Entorno** ($\\text{TA}$ en horas) | Adaptabilidad |
|  | **Número de Pasos de Instalación/Configuración** ($\\text{NPI}$) | Capacidad para ser Instalado |
| **Protección** (Parte de Seguridad) | **Porcentaje de Implementación de Controles de Acceso** ($\\%\\text{CA}$) | Control de Acceso |
|  | **Existencia de Pistas de Auditoría/Trazabilidad** ($\\text{PAT}$) | No Repudio / Responsabilidad |


**2\. Tabla de Características de Calidad vs. Indicadores y Umbrales**

Open RMF permite compartir información y facilitar la interoperabilidad de flotas de robots y las infraestructuras físicas subyacentes. En este contexto, se utlizan un conjunto de protocolos e interfaces de comunicación estandarizados que facilitan el intercambio de datos, señales de control e información adicional  entre robots, sensores, actuadores y otros dispositivos. Por lo expuesto, se puede consierar que Open-RMF es un software de misión crítico y bajo dicho contexto se deben definir los umbrales aceptables de cada una de las características aplicables a nuestro caso.

 A continuación, se presentan los valores como **recomendados** basadas en buenas prácticas de la industria para un software de calidad aceptable.

| Característica | Indicador / Métrica | Fórmula de Cálculo / Descripción | Umbral Aceptable (Sugerido) |
| :---- | :---- | :---- | :---- |
| **Adecuación Funcional** | $\\%$ Cobertura de Requisitos (Completa) | $\\frac{\\text{No. Requisitos implementados}}{\\text{No. Requisitos totales}} \\times 100$ | $\\ge 98\\%$ |
| **Eficiencia de Desempeño** | Tiempo de Respuesta (TR) | Tiempo transcurrido desde la solicitud hasta la respuesta (promedio) | $\\text{TR} \\le 1.000\\text{ ms}$ (para web interactivo) |
| **Compatibilidad** | $\\%$ Interoperabilidad Exitosa | $\\frac{\\text{No. Interacciones exitosas}}{\\text{No. Interacciones totales}} \\times 100$ con sistemas clave | $\\ge 99\\%$ |
| **Capacidad de Interacción** | Tiempo para Realizar Tarea (TTRT) | Tiempo promedio que un usuario nuevo tarda en completar una tarea clave. | Depende de la tarea, buscar $\\text{TTRT}$ bajo. |
| **Fiabilidad** | Densidad de Defectos (DLOC) | $\\frac{\\text{No. Defectos en producción}}{\\text{Líneas de Código Funcionales} (\\text{KLOC})} \\times 1000$ | $\\le 0.5$ defectos por KLOC |
|  | Tiempo Medio Entre Fallos ($\\text{MTBF}$) | $\\text{MTBF} \= \\frac{\\text{Tiempo total de operación}}{\\text{No. Fallos}}$ | El mayor valor posible, e.g., $\\ge 1.000\\text{ horas}$ |
| **Seguridad** | Tasa de Vulnerabilidades Críticas/Altas | $\\text{No. Vulnerabilidades Críticas/Altas}$ (CVSS $\\ge 7.0$) | $= 0$ |
| **Mantenibilidad** | Complejidad Ciclomática ($\\text{CC}$) | Número de caminos linealmente independientes en el código (por función/método). | $\\text{CC} \\le 10$ (Promedio); $\\text{CC} \\le 20$ (Máximo por función) |
|  | Índice de Mantenibilidad ($\\text{IM}$) | Índice compuesto que combina $\\text{CC}$, líneas de código, comentarios y Halstead. | $\\text{IM} \\ge 65$ (Ideal $\\ge 85$) |
| **Flexibilidad** (Portabilidad) | Porcentaje de Código Dependiente de Plataforma | $\\frac{\\text{No. Líneas de código no portables}}{\\text{No. Total de líneas de código}} \\times 100$ | $\\le 5\\%$ (Cuanto menor, mejor) |
| **Protección** (Seguridad) | Cobertura de Controles de Acceso | $\\frac{\\text{No. Funciones con control de acceso implementado}}{\\text{No. Funciones que lo requieren}} \\times 100$ | $\\ge 100\\%$ para funciones sensibles |


**3\. Herramientas Open Source para Métricas de Calidad**

Para realizar el análisis de calidad, dentro de njuestro contexto se ha considerado el uso de herramientas open source gracias a su acceso, disponibilidad y caracterísiticas relevantes de sus funcionalidades técnicas para su uso. Las herramientas open source permiten automatizar la obtención de métricas de calidad, especialmente en proyectos con múltiples lenguajes como Open-RMF.

| Característica de Calidad | Herramienta Open Source Sugerida | Tipo de Análisis | Lenguajes Soportados (Relevantes) |
| :---- | :---- | :---- | :---- |
| **Adecuación Funcional** | **Jest / Pytest / GoogleTest** | Pruebas Unitarias / Regresión | TypeScript/JavaScript, Python, C++ |
|  | **Selenium / Cypress** | Pruebas de Interfaz de Usuario/Sistema | Web (TypeScript/JavaScript) |
| **Eficiencia de Desempeño** | **JMeter / Locust** | Pruebas de Carga/Estrés | Agnostic, mide rendimiento del sistema. |
|  | **cProfile (Python) / Valgrind (C++)** | Profiling (Uso de Recursos) | Python, C++ |
| **Compatibilidad** | **BrowserStack (Limited Free Plan)** | Pruebas de Navegadores/Dispositivos | Web |
| **Fiabilidad** | **SonarQube / Code Climate** (Open Source Core) | Análisis Estático (Detección de *Bugs*) | C++, Python, TypeScript, JavaScript |
| **Seguridad** | **OWASP ZAP** (Zed Attack Proxy) | Pruebas Dinámicas de Seguridad (DAST) | Web/Agnóstico |
|  | **Bandit** | Análisis Estático de Seguridad (SAST) | Python |
|  | **ESLint / TSLint (con plugins de seguridad)** | Análisis Estático de Seguridad | TypeScript, JavaScript |
| **Mantenibilidad** | **SonarQube / ESLint / Pylint** | Análisis Estático (CC, IM, Acoplamiento, Cohesión) | C++, Python, TypeScript, JavaScript |
|  | **radon** | Métricas de Complejidad | Python |
|  | **Cloc / Loc (Lineas de Código)** | Tamaño del Código / Comentarios | Todos |


**4\. Cuadro Comparativo de Herramientas**

El equipo definido el uso de tres herramientas que cubren la mayoría de los lenguajes y características de calidad definidos para nuestro caso:
* **SonarQube (Community)** (pruebas de Fiabilidad, Seguridad, Mantenibilidad)
* **Pytest/Jest** (pruebas unitarias)
* **JMeter/Locust** (pruebas de rendimiento).

| Herramienta Open Source | Característica de Calidad Cubierta | Métricas Principales Obtenidas | Lenguajes Soportados |
| :---- | :---- | :---- | :---- |
| **SonarQube (Community)** | Fiabilidad, Seguridad, Mantenibilidad | **IM** (Índice de Mantenibilidad), **CC** (Complejidad Ciclomática), Densidad de *Bugs* y Vulnerabilidades, **Duplicación** de Código. | C++, Python, TypeScript, JavaScript, y muchos más. |
| **Pytest / Jest** (Tests Unitarios) | Adecuación Funcional, Fiabilidad | **% Cobertura de Código** (Líneas, Ramas), **Tasa de Éxito de Pruebas**, Fallos en Pruebas. | Python (Pytest), TypeScript/JS (Jest) |
| **JMeter / Locust** (Pruebas de Carga) | Eficiencia de Desempeño | **TR** (Tiempo de Respuesta), **TPS** (Transacciones por Segundo), **Latencia**, Errores por Carga. | Agnostic (Simula usuarios contra la URL/API) |
| **Bandit** | Seguridad | **Vulnerabilidades de Seguridad** (Gravedad Alta, Media, Baja) específicas de Python. | Python |
| **Cloc** | Mantenibilidad (Tamaño) | **LOC** (Líneas de Código), $\\text{Líneas en Blanco}$, $\\text{Líneas de Comentarios}$. | Todos |

**5\. Obtención de Métricas**

Para la obtención de las métricas de análisis estático y dinámico, utilizando las fuentes (rmf-web-main.zip y rmf\_demos-main.zip), se proporcionan los **pasos recomendados** siguientes:

### **Pasos para el Análisis Estático (Mantenibilidad, Fiabilidad, Seguridad)**

1. **Instalar SonarQube (Comunidad):** Es la forma más completa de obtener métricas estáticas en múltiples lenguajes.  
2. **Descomprimir los ZIP:** Extraiga rmf-web-main.zip y rmf\_demos-main.zip.  
3. **Configurar el escáner de SonarQube:** Ejecute el escáner de SonarQube en las carpetas de ambos proyectos.  
   * **Métrica a obtener:** **Índice de Mantenibilidad ($\\text{IM}$)**, **Complejidad Ciclomática ($\\text{CC}$)**, **Densidad de Defectos**, y **Vulnerabilidades de Seguridad**.

### **Pasos para el Análisis Dinámico y Funcional (Adecuación Funcional, Desempeño)**

1. **Ejecutar las Aplicaciones:** Necesita tener ambas aplicaciones (especialmente **rmf-web** y el backend de **rmf\_demos**) en un entorno operativo.  
2. **Pruebas de Desempeño con JMeter/Locust:**  
   * **Métrica a obtener:** **Tiempo de Respuesta Promedio ($\\text{TR}$)** y **TPS** (Transacciones por Segundo) en los *endpoints* críticos.  
3. **Pruebas de Cobertura de Código:**  
   * Si existen pruebas unitarias (muy probable en proyectos de esta índole): ejecute pytest para Python y jest para TypeScript/JavaScript.  
   * **Métrica a obtener:** **% Cobertura de Pruebas Unitarias** (líneas, ramas).
