MRASL Lab Demo
==============

Cette section a pour but de permettre de facilement mettre en place la démonstration pour le
laboratoire.


Installation des ancres
-----------------------

Pour une précision optimale, il est recommandé d'utiliser les 8 ancres LPS. Par contre, la demonstration
fonctionne aussi avec seulement 6.

1 - Installer les ancres sur les 4 supports. L'ancre du bas devrait être à environ 20cm du sol et celle du haut à environ 2m.

.. todo:: PHOTO

2 - Disposer les ancres aux 4 coins de l'arène

3 - Mesurer la position des ancres

    - Pour faciliter cette étape, je recommande d'utiliser le VICON

4 - Mettre à jour la position des ancres à l'aide de l'interface Bitcraze

5 - Alimenter les ancres

    - Le plus facile est d'utliser des power banks


Lancer la démonstration
-----------------------

La démonstration a été testée et optimisée pour 5 drones. Cependant, elle devrait fonctionner avec n'importe quel nombre.

.. note:: Il est recommandé de tester les drones individuellement pour s'assurer qu'ils volent bien.

1 - Placer les drones dans l'arène

    Leur position initiale n'est pas importante. Il faut seulement qu'ils soient tous à au moins 1 mètre
    de distance l'un de l'autre

2 - Lancer le serveur

    Le projet est installé sur l'ordi...

    .. todo:: ORDI DU LABO

    ::

        $ roslaunch swarm_manager launch_swarm.launch


    .. warning::

        N'oublier pas d'avoir ``roscore`` qui roule dans un terminal

3 - Démarrer la démonstration

    ::

        $ cd projects/crazyflie_charles/demos
        $ python mrasl_demo.py
