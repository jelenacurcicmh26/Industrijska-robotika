# xArm7 — upravljanje na nivou momenata u MuJoCo-u

Tri upravljačka zakona rastuće složenosti, na nivou momenata u zglobovima, primenjena na
robotsku ruku UFACTORY xArm7 sa sedam stepeni slobode, izgrađena korak po korak.
**MuJoCo** je simulator koji igra ulogu realnog sistema, **Pinocchio** računa dinamički
model na koji se upravljanje oslanja, a njih dva se u svakom koraku porede međusobno.

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

Pinocchio nema Windows paket na PyPI-ju i mora da se instalira sa conda-forge kanala.
Paket koji se na PyPI-ju zove `pinocchio` je nepovezan i napušten projekat — instalacija
preko pip-a daje prazan modul bez funkcije `buildModelFromMJCF`.

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

Moment u svim zglobovima je nula, pa ruka pada. Suština nije u padanju, nego u tome što
se u ovom koraku gravitacioni moment koji Pinocchio računa iz MJCF opisa poredi sa
MuJoCo-vom veličinom `qfrc_bias`. Slaganje je reda 1e-13 N·m, i to je ono što opravdava
sve što sledi.

## 2. korak — PD regulator sa kompenzacijom gravitacije

$$\tau = g(q) + K_p (q_d - q) + K_d(\dot q_d - \dot q)$$

Gravitacija se poništava unaprednom kompenzacijom, a ostatak preuzima sistem opruge i
prigušenja. Ovo radi, ali efektivna inercija koju zglob „oseća" zavisi od konfiguracije —
ispruženu ruku je mnogo teže pokrenuti nego skupljenu — pa je jedno pojačanje premeko u
jednoj pozi, a prenaglo u drugoj. Otuda sedam ručno podešenih parova pojačanja.

## 3. korak — inverzna dinamika (computed torque)

$$\tau = M(q)\,a_{\text{ref}} + C(q,\dot q)\dot q + g(q) + D\dot q + f_c \tanh(\dot q / \varepsilon)$$

$$a_{\text{ref}} = \ddot q_d + K_d(\dot q_d - \dot q) + K_p(q_d - q)$$

Umesto da gura jače kada greška poraste, upravljanje računa moment koji je ruci zaista
potreban za kretanje koje se traži, i dodaje malu korekciju. Uvrštavanjem u jednačine
dinamike ostaje

$$\ddot e + K_d \dot e + K_p e = 0$$

što je isti sistem drugog reda za svaki zglob i u svakoj pozi. Pojačanja prestaju da budu
sedam ručno podešenih parova i postaju podešavanje polova: jedna sopstvena učestanost
`OMEGA = 20 rad/s`, kritično prigušenje, a `Kp = OMEGA²` i `Kd = 2·OMEGA` slede iz nje.

---

## Rezultati

Sva tri zakona, ista putanja, ista pojačanja, jedan pored drugog:

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

## Deo koji nije prošao očekivano

Zanimljiv je srednji red te tabele. *Napredniji* zakon, zasnovan na modelu, pratio je
**tri puta lošije** od jednostavnog PD regulatora koji je trebalo da nadmaši.

Udžbenički izvod pretpostavlja da je robot skup krutih tela sa masom i ništa više. Ali
`xarm7.xml` svakom zglobu ruke dodeljuje i viskozno prigušenje (10/10/5/5/5/2/2 N·m·s/rad)
i Kulonovo trenje (`frictionloss`) od 1 N·m. Te sile su stvarne u simulaciji, ali nisu deo
izraza $M\ddot q + C\dot q + g$ — MuJoCo ih drži u `qfrc_passive`, a ne u `qfrc_bias` — pa
ih model krutog tela koji Pinocchio gradi iz istog fajla ne vidi, i udžbenički zakon ih
nikada ne poništava.

Nisu u pitanju zanemarljive veličine:

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/torque-breakdown-dark.svg">
    <img src="docs/figures/torque-breakdown-light.svg" width="880" alt="RMS doprinos svakog člana momentu, po zglobovima">
  </picture>
</p>

U zglobovima 1, 3, 5 i 7 — rotacionim zglobovima čije ose leže približno duž ruke, pa
gotovo i ne nose gravitaciono opterećenje — članovi trenja su *veći* od celokupnog
doprinosa krutog tela. Samo su zglobovi 2 i 4 zaista pod dominantnim uticajem gravitacije.

Linearizacija povratnom spregom je neoprostiva na način na koji PD regulator nije: njena
cela premisa je da tačno poništava dinamiku sistema, pa sve što ostane neponišteno ide
pravo u grešku praćenja. Sistem opruge i prigušenja nikada nije ni tvrdio da poznaje
sistem, i isto to trenje tiho apsorbuje kao dodatno prigušenje.

> Vredi reći otvoreno: vrednost od 0.029° dolazi iz simulacije u kojoj su model koji
> upravljanje koristi i sam simulirani sistem generisani iz istog XML fajla. To pokazuje
> da linearizacija povratnom spregom radi; to nije tvrdnja o ponašanju na realnom robotu.

## Zašto zadata putanja mora da bude glatka

Inverzna dinamika unapred prosleđuje željeno ubrzanje $\ddot q_d$, pa zadata veličina ne
može da bude odskočna funkcija — skokovi iz 2. koraka između poza zahtevaju beskonačno
ubrzanje i prosto dovedu pogone u zasićenje.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/step-vs-quintic-dark.svg">
    <img src="docs/figures/step-vs-quintic-light.svg" width="860" alt="Zadati moment pri odskočnoj i pri polinomijalnoj putanji">
  </picture>
</p>

Sa odskočnom zadatom vrednošću isti zakon dovodi pogone u zasićenje u 7% koraka i dostiže
grešku od 74°. Zato su međutačke povezane polinomima petog reda, koji počinju i završavaju
u mirovanju sa nultim ubrzanjem, i primaju proizvoljne početne uslove, tako da promena
cilja usred kretanja ostaje $C^2$ neprekidna:

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

## Ponovno generisanje svega

```bash
python tools/benchmark.py       # results.csv i svi grafici
python tools/record_media.py    # sve animacije i slike
```

Obe skripte rade bez otvaranja prozora. Svaki broj i svaki grafik u ovom dokumentu dolazi
iz njih, tako da ništa ovde ne može da se razmimoiđe sa onim što upravljanje zaista radi.

## Naredni koraci

- **Upravljanje u prostoru zadatka** — dodavanje matrice inercije u prostoru zadatka
  $\Lambda = (JM^{-1}J^\top)^{-1}$ i dinamički doslednog projektora nulti prostor, čime bi
  se dovršio [`controllers/impedance_control.py`](controllers/impedance_control.py).
- **Adaptivno upravljanje (Slotine–Li)** — iskorišćavanje linearnosti po inercijalnim
  parametrima preko funkcije `computeJointTorqueRegressor` iz Pinocchio-a, tako da robot
  sam identifikuje ono što se u ovom projektu čita iz XML fajla. S obzirom na gornji nalaz
  o trenju, ovo je očigledan nastavak.
- **Posmatrač spoljašnjeg momenta zasnovan na impulsu** — detekcija sudara i vođenje rukom
  bez senzora sile.
- **Upravljanje zasnovano na kvadratnom programiranju** — određivanje momenta uz poštovanje
  stvarnih ograničenja pogona, granica zglobova i ograničenja za izbegavanje prepreka.

## Poreklo modela

Opis robota u direktorijumu [`models/xarm7/`](models/xarm7/) **nije moj rad**. Preuzet je
iz [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) i izveden je iz
javno dostupnog [xArm7 URDF opisa](https://github.com/xArm-Developer/xarm_ros) kompanije
UFACTORY. Copyright © 2018 UFACTORY Inc.; puni uslovi, koji važe za taj direktorijum, nalaze
se u [`models/xarm7/LICENSE`](models/xarm7/LICENSE). Sve izvan `models/` je moj rad.

## Literatura

- Siciliano, Sciavicco, Villani, Oriolo — *Robotics: Modelling, Planning and Control*, pogl. 8
  (upravljanje inverznom dinamikom).
- Khatib — *A Unified Approach for Motion and Force Control of Robot Manipulators: The
  Operational Space Formulation*, IEEE J. Robotics and Automation, 1987.
- Slotine, Li — *On the Adaptive Control of Robot Manipulators*, IJRR, 1987.
- [MuJoCo dokumentacija](https://mujoco.readthedocs.io/) — tok proračuna, razlika između
  `qfrc_bias` i `qfrc_passive`.
- [Pinocchio dokumentacija](https://stack-of-tasks.github.io/pinocchio/) — `crba`,
  `nonLinearEffects`, `rnea`.
