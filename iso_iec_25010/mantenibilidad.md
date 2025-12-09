# **Mantenibilidad**

La mantenibilidad es una característica de calidad orientado a conocer
el grado en el que un producto de software puede ser modificado de forma
efectiva y eficiente, debido a necesidades evolutivas, correctivas o
mejoras. Las sub-características de modularidad, reusabilidad,
analizabilidad, modificabilidad y capacidad de pruebas son aspectos a
considerar durante la evaluación de la mantenibilidad del producto de
software.


| Caracterítica | Descripción |
| ----------- | ----------- |
| **Modularidad** | Métricas de acoplamiento (coupling), análisis de                      dependencias entre módulos, cohesión de clases, grafos de dependencias |
| **Reusabilidad** | Detección exhaustiva de código duplicado, métricas de duplicación por archivo/proyecto, análisis de clones de código |
|  **Analizabilidad**|Complejidad ciclomática, complejidad cognitiva,                         métricas de líneas de código, comentarios, dashboards interactivos, tendencias históricas|
|  **Modificabilidad**| Code smells, deuda técnica cuantificada en tiempo,                     issues clasificados por severidad, hotspots de seguridad y mantenibilidad|
| **Capacidad de Prueba** | Cobertura de pruebas unitarias e integración, pruebas duplicadas, complejidad de métodos testeables, análisis de tests|
  
Para identificar el grado de mantenibilidad del producto de software
hacemos uso de las métricas siguientes:

| **Métricas** | **Sub-características** |
| ----------- | ----------- |
| Complejidad ciclomática | Modificabilidad|
| Duplicación de código|  Reusabilidad|
| Cobertura de pruebas |Capacidad de Prueba|
| Acoplamiento y cohesión| Modularidad|
| Deuda técnica|  Analizabilidad|
|Violaciones de estándares | Todas|

Para obtener las métricas antes señaladas, podemos hacer uso de una o
varias herramientas open source, tales como:
1. SonarQube
2. PMD
3. Checkstyle
4. Jdepend
5. ESLint/Pylint/RuboCop

El cuadro comparativo de las herramientas open source SonarQube, PMD,
Checkstyle, JDepend, ESLint/Pylint/RuboCop permite identificar las
herramientas más adecuadas para analizar el cumplimiento de la
mantenibilidad.

![Cuadro comparativo Mantenibilidad](mantenibilidad_diagrama.png)

Basados en el cuadro comparativo, la práctica en el sector de calidad de
software y para garantizar la evaluación integral de mantenibilidad
según ISO 25010, la mejor estrategia es combinar el uso de SonarQube
como plataforma central y Linter específico del lenguaje para la
realimentación inmediata durante desarrollo.

