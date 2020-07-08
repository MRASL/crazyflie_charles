# Inventaire
## Matériel
Au 6/06/2020

|     Matériel      |  Quantité 	|
|:-----------------:|:-----------:|
|Piles              |   	26      |
|Chargeur usb       |   	17      |
|LPS Nodes          |      8      |
|Crazyradio         |   	 3      |
|Spare motor        |   	10      |
|Spare motor support|   	19      |
|Spare propeller A  |   	30      |
|Spare propeller B  |   	31      |
|Battery holder     |   	10      |
|Long headers       |   	20      |
|Micro USB Cable*   |   	12      |

\*Certains cables peuvent seulement charger


## Crazyflies
|   Id 	|    Address    | Status |  Roll trim  | Pitch trim  |
|:-----:|:-------------:|:------:|:-----------:|:-----------:|
|   0   | 0xE7E7E7E700  |   OK   |     0.0     |     0.0     |
|   1   | 0xE7E7E7E701  |   OK   |    -1.5     |     0.8     |
|   2   | 0xE7E7E7E702  |   OK   |    -1.8     |     0.0     |
|   3   | 0xE7E7E7E703  |   OK   |    -1.6     |     0.0     |
|   4   | 0xE7E7E7E704  |   OK   |    -1.0     |     1.6     |
|   5   | 0xE7E7E7E705  |   OK   |    -1.4     |     0.4     |
|   6   | 0xE7E7E7E706  |   OK   |    -1.0     |    -0.4     |
|   7   | 0xE7E7E7E707  |   OK   |    -0.4     |     0.2     |


# To Do
## Rerche sur le path planning
  - [Article 1](https://ieeexplore.ieee.org/document/8598938)
  - [Video 1](https://www.youtube.com/watch?v=ZN2e7h-kkpw&feature=emb_title)
  - [Article 2](https://arxiv.org/pdf/1909.05150.pdf)
  - [Video 2](https://www.youtube.com/watch?v=N4rWiraIU2k&feature=emb_title)

## Recherche sur le lightouse
  - Résolution: absolute error < 10cm, relative error: mm
  - n de cf
  - Coût total
  - Information disponible: x, y, z, yaw
  - Vidéos
    - [One station](https://www.youtube.com/watch?v=yhx0BSGxh5Q)
    - [Yaw exemple](https://www.youtube.com/watch?v=NHdlHIq_ce0)
    - [Moving in a house](https://www.youtube.com/watch?v=gQkrPM6UUbs)
    - [Swarm](https://www.youtube.com/watch?v=BggnSDj3baE)


## Liste du matériel
  - VICON
    - Marker
    - Marker deck
  - Spare parts bundle
  - LED ring deck
  - Lightouse
    - Lightouse deck
    - Base station
    - Support

## Objectifs
  - [x] Vol en formation
    - [x] Supporter plusieurs formations
      - [x] Carre
      - [x] Cercle
      - [x] Pyramide
    - [x] Changement entre les formation
      - [x] En utilisant l'algorithme de planification de trajet
    - [x] Pilotage en formation
      - [x] x, y, z, yaw
    - [x] Aller en formation a partir de pos inital random
    - [x] Determiner les cf de la formation selon le nombre dispo
  - [ ] Choregraphie
    - [ ] Executer une sequence preenregistre