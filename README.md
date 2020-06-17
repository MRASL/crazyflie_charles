#  Contrôle d'un essaim de drône

Le but de ce projet est de pouvoir contrôler la formation d'un essaim de drônes à l'aide d'un interface graphique. Un tel projet poura être utilisé à des fin de démonstration et pour tester, en pratique, des algorithmes.

## Liens intéressants

- [Documentation du labo](https://mrasl.gitbooks.io/documentation/UAV/bitcraze-crazyflie.html)
- [Tutoriel Git](https://git-scm.com/book/en/v2/Git-Basics-Getting-a-Git-Repository)
- [Google docstring](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html)
- [Python Style Guide](https://www.python.org/dev/peps/pep-0008/#naming-conventions)

### Recherches
- [Documentation](/Documentation/Summary.md)
- [Rercherche - Bitcraze](https://www.bitcraze.io/portals/research/)
- [Multirobot System - blog](https://www.bitcraze.io/2017/06/towards-persistent-adaptive-multi-robot-systems/)
- [Modquad - blog](https://www.bitcraze.io/2017/11/modquad-self-assemble-flying-structures/)

### Repo
- [Bitcraze](https://github.com/bitcraze)
- [Crazyflie-tools](https://github.com/blandry/crazyflie-tools)
  - Used to avoid obstacles (see Landry B. thesis)
- [CrazySwarm](https://github.com/USC-ACTLab/crazyswarm) 
  - 49 CF controlled with Vicon
- [Tunnel Mod](https://github.com/resibots/crazyflie-firmware/)
  - Peer2Peer communicatio between CF


### Crazyflie
- [Crazyflie 2.1](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/)
  - [Controler types](https://www.bitcraze.io/2020/02/out-of-control/)
  - [Drone Dynamics](https://www.bitcraze.io/2018/11/demystifying-drone-dynamics/)
  - [Position Control Architecture](https://www.bitcraze.io/2016/05/position-control-moved-into-the-firmware/)
  
- [Getting started](https://www.bitcraze.io/documentation/start/)
  - [First setup](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/)
  - [LPS System](https://www.bitcraze.io/documentation/tutorials/getting-started-with-loco-positioning-system/)
  - [Expansion Decks](https://www.bitcraze.io/documentation/tutorials/getting-started-with-expansion-decks/)
  - [Development](https://www.bitcraze.io/documentation/tutorials/getting-started-with-development/)

- [Bitcraze](https://www.bitcraze.io/)
  - [Wiki](https://wiki.bitcraze.io/)
  - [Blog](https://www.bitcraze.io/blog/)
  - [Forum](https://forum.bitcraze.io/)
  - [Crazyflie-lib-doc](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/python_api/)

### ROS
- [Livre ROS](https://books.google.ca/books?id=skFPDwAAQBAJ&pg=PA339&lpg=PA339&dq=crazyflie+lib+scan&source=bl&ots=fgtFKHcJSd&sig=ACfU3U0iAfU0DtVE7Bun8h29GCLQc0aXOg&hl=fr&sa=X&ved=2ahUKEwiz3O-5grTpAhUBU98KHVRLDw4Q6AEwAnoECAoQAQ#v=onepage&q&f=false)
- [Tutoriels](http://wiki.ros.org/ROS/Tutorials)
- [Autre live](https://www.scribd.com/document/360234366/Robot-Operating-System-ROS-The-Complete-Reference-Volume-2)


## Structure des fichiers

  - examples
    - Example scripts
  - include
    - currently empty
  - launch
    - All .launch files
  - scripts
    - All scripts related to swarm managment
  - srv
    - Services
  - trajectory_planning
    - Scripts related to trajectory planning 