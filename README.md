# 🤖 IntoTheDeep_final – Codul echipei InfinityBolts FTC #19093

Acest repository conține codul sursă al echipei InfinityBolts FTC #19093 pentru sezonul 2023-2024 *CENTERSTAGE* (tema: **INTO THE DEEP**) din cadrul competiției FIRST Tech Challenge (FTC). Proiectul este scris în Java și utilizează Road Runner pentru navigație autonomă, MeepMeep pentru simulări și multiple module hardware pentru a optimiza performanța robotului pe teren.

---

## 🧠 Arhitectura Software

Codul este modularizat și organizat în funcție de următoarele componente majore:

### 1. **Moduri de operare (OpModes)**
- `Autonom` – Clasa de bază pentru autonom (ex: `BlueLeft`, `RedRight` etc.), fiecare cu strategii diferite de parcare, poziționare pe linia de aliniere și depozitare de **sample-uri**.
- `TeleOp` – Codul pentru controlul manual al robotului, cu suport pentru mecanum drive, manipulare braț, intake, outtake și ajustare dinamică a vitezei.

### 2. **Sisteme hardware personalizate**
- `HardwareMap_IB` – Clasa centrală pentru inițializarea motoarelor, servourilor, senzorilor și encoderelor.
- `DriveTrain_IB` – Implementare mecanum drive cu ajustări pentru precizie și alunecare pe suprafață.
- `Arm_IB` – Controlul brațului articulat și al sistemului de ridicare.
- `Intake_IB` și `Outtake_IB` – Sisteme pentru colectare și depozitare a elementelor de joc (sample-uri).
- `Turret_IB` – Modul de rotire pentru aliniere automată cu ținta.

### 3. **Traiectorii și simulare**
- `RoadRunner` – Folosit pentru definirea traiectoriilor spline, constante de accelerație și poziționare precisă în autonom.
- `MeepMeepTesting` – Vizualizări pentru verificarea logicii autonome fără a depinde de robotul fizic.

---

## 📊 Strategii de joc

### ♟️ În Autonom:
- Detectarea poziției sample-ului folosind **AprilTags** și/sau analiza de culoare.
- Depozitarea strategică în **Backboard** sau plasarea în poziția centrală pentru puncte rapide.
- Parcare în zona aliatului pentru puncte bonus.
- Traiectorii optimizate pentru coliziuni minime cu elemente de decor (bariere, coastă etc.).

### 🎮 În TeleOp:
- Sistem mecanum drive cu **mod slow-motion** pentru control fin în apropierea țintelor.
- Braț articulat cu presetări pentru nivelurile **Low, Mid, High**.
- Operare duală (driver + operator) cu scheme de control intuitive.
- Reconfigurare automată a poziției robotului după colectare.

---

## 🧰 Tehnologii și echipamente utilizate

- **Road Runner v1.0** – pentru localizare și deplasare autonomă.
- **GoBilda 435RPM motors** – cu encoder intern.
- **Servouri Axon MAX** – pentru prindere rapidă și precizie.
- **Odometry GoBilda** – cu tensionare integrată pentru detecție poziție.
- **Camera Limelight 3A** (FTC-legală) – pentru identificare și urmărire obiecte.
- **Control Hub + Expansion Hub** – pentru conectivitate extinsă și fiabilitate.

---

## ⚙️ Instalare rapidă

1. **Clonează repository-ul**:
   ```bash
   git clone https://github.com/InfinityBolts-FTC19093/IntoTheDeep_final.git
   ```

2. **Deschide în Android Studio**:
    - Selectează „Open an existing project”.
    - Alege folderul `IntoTheDeep_final`.

3. **Build & Deploy**:
    - Conectează Control Hub-ul prin USB sau Wi-Fi.
    - Selectează modul (`TeleOp` / `Autonom`) și rulează aplicația.

---

## 📄 Licență

Acest proiect este publicat sub licența **BSD-3-Clause-Clear**. Pentru mai multe detalii, consultă fișierul [LICENSE](LICENSE).

---

## 👥 Despre echipă

**InfinityBolts #19093** este o echipă din Arad, România, pasionată de robotică, educație STEM și inovație. Avem o abordare interdisciplinară care combină ingineria mecanică, programarea avansată și designul 3D, obținând rezultate remarcabile atât la nivel național, cât și internațional.
