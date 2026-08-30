# xArm7 — upravljanje na nivou momenata u MuJoCo-u

Tri upravljačka zakona na nivou momenata u zglobovima, primenjena na ruku UFACTORY xArm7
sa sedam stepeni slobode, građena jedan na drugom. MuJoCo računa fiziku i igra ulogu
realnog robota. Pinocchio računa model krutog tela koji upravljanje koristi. Svaki korak
prvo proveri to dvoje jedno prema drugom, pa tek onda se osloni na njih.

<p align="center">
  <img src="docs/media/hero.gif" width="620" alt="xArm7 prati ciklus međutačaka pod upravljanjem inverznom dinamikom">
</p>

![Python](https://img.shields.io/badge/python-3.12-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.12-orange)
![Pinocchio](https://img.shields.io/badge/Pinocchio-4.1-green)

Projekat iz predmeta *Industrijska robotika*, doktorske studije, Fakultet tehničkih nauka,
Univerzitet u Novom Sadu.

**🇬🇧 [This page in English](README.md)**

---

## Tri koraka

| Korak | Skripta | Upravljački zakon |
|---|---|---|
| 1 | [`step1_gravity_only.py`](step1_gravity_only.py) | `τ = 0` |
| 2 | [`step2_PD_with_gravity_compensation.py`](step2_PD_with_gravity_compensation.py) | `τ = g(q) + Kp·e + Kd·ė` |
| 3 | [`step3_computed_torque.py`](step3_computed_torque.py) | `τ = M(q)·a_ref + C(q,q̇)·q̇ + g(q) + trenje` |

## Pokretanje

```bash
conda env create -f environment.yml
conda activate robotika

python step1_gravity_only.py
python step2_PD_with_gravity_compensation.py
python step3_computed_torque.py
```

Pinocchio mora da se instalira sa conda-forge kanala, jer za Windows ne postoji paket na
PyPI-ju. Treba paziti na PyPI paket koji se zove `pinocchio`: to je drugi, napušten
projekat, i instalacija preko pip-a daje prazan modul bez funkcije `buildModelFromMJCF`.

Tasteri, kada prozor simulatora ima fokus:

| Taster | Radnja |
|---|---|
| `SPACE` | pauza / nastavak |
| `R` | povratak u početnu pozu |
| `A` | uključivanje automatskog ciklusa kroz međutačke |
| `1` `2` `3` | prelazak u pozu 1 / 2 / 3 |
| `C` | promena upravljačkog zakona (samo 3. korak) |
| `E` | resetovanje statistike greške (samo 3. korak) |
| `ESC` | izlaz |

---

## 1. korak — samo gravitacija

<p align="center">
  <img src="docs/media/step1-gravity.gif" width="520" alt="Ruka pada pod dejstvom gravitacije bez upravljačkog momenta">
</p>

Moment u svim zglobovima je nula, pa ruka padne. Samo padanje nije poenta. Ovaj korak
služi da se gravitacioni moment koji Pinocchio računa iz MJCF fajla uporedi sa MuJoCo-vom
veličinom `qfrc_bias`. Slaganje je reda 1e-13 N·m. I 2. i 3. korak polaze od toga da
Pinocchio-ov model opisuje istog robota kog MuJoCo simulira, pa to vredi jednom proveriti.

## 2. korak — PD regulator sa kompenzacijom gravitacije

$$\tau = g(q) + K_p (q_d - q) + K_d(\dot q_d - \dot q)$$

Unapredna grana poništava gravitaciju, a ostatak preuzimaju opruga i prigušenje. Radi
sasvim solidno, ali su pojačanja nezgodna za podešavanje. Koliku inerciju zglob „oseća"
zavisi od konfiguracije: ispruženu ruku je mnogo teže pokrenuti nego skupljenu. Jedna
vrednost pojačanja ispadne premeka u jednoj pozi, a prenagla u drugoj, i zato u fajlu
stoji sedam ručno podešenih parova.

## 3. korak — inverzna dinamika (computed torque)

$$\tau = M(q)\,a_{\text{ref}} + C(q,\dot q)\dot q + g(q) + D\dot q + f_c \tanh(\dot q / \varepsilon)$$

$$a_{\text{ref}} = \ddot q_d + K_d(\dot q_d - \dot q) + K_p(q_d - q)$$

Umesto da gura jače kako greška raste, ovde se računa moment koji je ruci stvarno potreban
za kretanje koje se od nje traži, pa se na to doda mala korekcija. Kada se to uvrsti u
jednačine dinamike, ostaje

$$\ddot e + K_d \dot e + K_p e = 0$$

isti sistem drugog reda za svaki zglob i u svakoj pozi. Pojačanja zato više nisu sedam
podešenih parova nego podešavanje polova. Bira se jedna sopstvena učestanost,
`OMEGA = 20 rad/s`, uzme se kritično prigušenje, a `Kp = OMEGA²` i `Kd = 2·OMEGA` slede
iz toga.

---

## Rezultati

Sva tri zakona na istoj putanji i sa istim pojačanjima:

<p align="center">
  <img src="docs/media/comparison.gif" width="900" alt="Tri upravljačka zakona na istoj putanji">
</p>

| Upravljački zakon | RMS greška | Maksimalna greška |
|---|---:|---:|
| PD + gravitacija (2. korak) | 0.992° | 3.29° |
| Inverzna dinamika, samo kruto telo | 2.979° | 11.66° |
| **Inverzna dinamika, pun model** | **0.029°** | **0.20°** |

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/tracking-error-dark.svg">
    <img src="docs/figures/tracking-error-light.svg" width="860" alt="Greška praćenja tokom jednog ciklusa, logaritamska skala">
  </picture>
</p>

Sirovi brojevi se nalaze u [`docs/results.csv`](docs/results.csv).

## Problem sa trenjem

Zanimljiv je srednji red te tabele. Zakon zasnovan na modelu, koji je trebalo da bude
poboljšanje, pratio je tri puta lošije od PD regulatora iz 2. koraka.

Udžbenički izvod posmatra robota kao skup krutih tela sa masom i ništa više. Ali
`xarm7.xml` svakom zglobu ruke dodeljuje i viskozno prigušenje (10/10/5/5/5/2/2 N·m·s/rad)
i Kulonovo trenje (`frictionloss`) od 1 N·m. Te sile deluju u simulaciji, ali nisu deo
izraza $M\ddot q + C\dot q + g$. MuJoCo ih drži u `qfrc_passive`, a ne u `qfrc_bias`, pa
ih model krutog tela koji Pinocchio gradi iz istog fajla nikada ne vidi, i upravljački
zakon ih nikada ne poništi.

Ispostavlja se da nisu male:

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/torque-breakdown-dark.svg">
    <img src="docs/figures/torque-breakdown-light.svg" width="880" alt="RMS doprinos svakog člana momentu, po zglobovima">
  </picture>
</p>

U zglobovima 1, 3, 5 i 7 članovi trenja su veći od celog doprinosa krutog tela. To su
rotacioni zglobovi čije ose leže približno duž ruke, pa gotovo i ne nose gravitaciono
opterećenje. Samo su zglobovi 2 i 4 zaista pod dominantnim uticajem gravitacije.

Linearizacija povratnom spregom je oko ovoga mnogo manje popustljiva nego PD. Ona radi
tako što tačno poništava dinamiku, pa sve što ne uspe da poništi ide pravo u grešku
praćenja. Opruga i prigušenje nikada nisu ni pretpostavili da poznaju sistem, i isto to
trenje jednostavno pokupe kao dodatno prigušenje.

Jedna ograda oko vrednosti od 0.029°: ovde su model koji upravljanje koristi i simulirani
sistem generisani iz istog XML fajla. Taj broj pokazuje da metoda radi, ali nije tvrdnja
o tome kako bi se ovo ponašalo na stvarnom robotu.

## Zašto zadata putanja mora da bude glatka

Upravljački zakon unapred prosleđuje željeno ubrzanje $\ddot q_d$, pa zadata veličina ne
može da bude odskočna. U 2. koraku se skakalo pravo iz jedne poze u drugu, što traži
beskonačno ubrzanje i samo dovede pogone u zasićenje.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/step-vs-quintic-dark.svg">
    <img src="docs/figures/step-vs-quintic-light.svg" width="860" alt="Zadati moment pri odskočnoj i pri polinomijalnoj putanji">
  </picture>
</p>

Sa odskočnom zadatom vrednošću isti zakon dovodi pogone u zasićenje u 7% koraka i dostiže
grešku od 74°. Zato su međutačke povezane polinomima petog reda. Oni počinju i završavaju
u mirovanju sa nultim ubrzanjem, i primaju proizvoljne početne uslove, pa pritisak na `2`
usred kretanja ka pozi 3 planira novu putanju od trenutnog stanja, bez skoka.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/quintic-profile-dark.svg">
    <img src="docs/figures/quintic-profile-light.svg" width="720" alt="Pozicija, brzina i ubrzanje jednog polinomijalnog segmenta">
  </picture>
</p>

---

## Struktura repozitorijuma

```
step1_gravity_only.py                    τ = 0 i provera gravitacije Pinocchio/MuJoCo
step2_PD_with_gravity_compensation.py    PD sa kompenzacijom gravitacije
step3_computed_torque.py                 inverzna dinamika, polinomijalne putanje, poređenje
controllers/impedance_control.py         impedansno upravljanje (u izradi)
tools/
    _sim.py            pokretanje simulacije bez prozora, oko upravljanja iz 3. koraka
    _svg.py            minimalni generator SVG grafika
    benchmark.py       pokreće upravljanja -> docs/results.csv + docs/figures
    record_media.py    renderovanje van ekrana -> docs/media
docs/
    figures/           grafici, svetla i tamna verzija
    media/             animacije i slike
    results.csv        brojevi iz tabela
models/xarm7/          opis robota (videti Poreklo modela)
```

## Ponovno generisanje grafika

```bash
python tools/benchmark.py       # results.csv i svi grafici
python tools/record_media.py    # sve animacije i slike
```

Nijedna od te dve skripte ne otvara prozor. Svaki broj i svaki grafik u ovom dokumentu
dolazi iz njih, tako da ništa ovde ne može neprimetno da se razmimoiđe sa onim što
upravljanje zaista radi.

U `environment.yml` namerno nema biblioteke za crtanje. Prevedeni delovi matplotlib-a ne
mogu da se učitaju pored ovog mujoco/pinocchio okruženja na Windows-u: svaki poziv za
iscrtavanje pukne sa greškom `0xc06d007f` u `matplotlib._path`, bez ikakve poruke. Zato
`tools/_svg.py` sam ispisuje SVG fajlove.

## Naredni koraci

- **Upravljanje u prostoru zadatka.** Dodati matricu inercije u prostoru zadatka
  $\Lambda = (JM^{-1}J^\top)^{-1}$ i dinamički dosledan projektor na nulti prostor, čime bi
  se dovršio [`controllers/impedance_control.py`](controllers/impedance_control.py).
- **Adaptivno upravljanje (Slotine–Li).** Dinamika je linearna po inercijalnim parametrima,
  a Pinocchio ima `computeJointTorqueRegressor`, pa bi robot mogao sam da identifikuje ono
  što se u ovom projektu čita iz XML fajla. Posle gornjeg nalaza o trenju, ovo je
  očigledna sledeća stvar za probati.
- **Posmatrač spoljašnjeg momenta zasnovan na impulsu,** za detekciju sudara i vođenje
  rukom bez senzora sile.
- **Upravljanje zasnovano na kvadratnom programiranju,** gde se moment određuje uz
  poštovanje stvarnih ograničenja pogona, granica zglobova i uslova za izbegavanje
  prepreka.

## Poreklo modela

Opis robota u direktorijumu [`models/xarm7/`](models/xarm7/) nije moj rad. Preuzet je iz
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), a izveden je iz
javnog [xArm7 URDF opisa](https://github.com/xArm-Developer/xarm_ros) kompanije UFACTORY.
Copyright © 2018 UFACTORY Inc. Puni uslovi stoje u
[`models/xarm7/LICENSE`](models/xarm7/LICENSE) i važe za taj direktorijum. Sve izvan
`models/` je moje.

## Literatura

- Siciliano, Sciavicco, Villani, Oriolo, *Robotics: Modelling, Planning and Control*, pogl. 8.
- Khatib, „A Unified Approach for Motion and Force Control of Robot Manipulators: The
  Operational Space Formulation", IEEE J. Robotics and Automation, 1987.
- Slotine i Li, „On the Adaptive Control of Robot Manipulators", IJRR, 1987.
- [MuJoCo dokumentacija](https://mujoco.readthedocs.io/), o toku proračuna i razlici
  između `qfrc_bias` i `qfrc_passive`.
- [Pinocchio dokumentacija](https://stack-of-tasks.github.io/pinocchio/), za `crba`,
  `nonLinearEffects` i `rnea`.
