# 🤖 IntoTheDeep_final – Proiect robotică FTC: Sezonul INTO THE DEEP (2024-2025)

Acest repository conține codul sursă al echipei InfinityBolts FTC #19093 pentru sezonul 2024-2025 al competiției **FIRST Tech Challenge (FTC)**, cu tema **INTO THE DEEP**. Proiectul este dezvoltat în Java și include funcționalități avansate de control autonom, navigație, simulare și operare manuală, optimizate pentru provocările specifice ale sezonului.

---

## 📁 Structura proiectului

- `FtcRobotController/` – Codul principal al aplicației care rulează pe Control Hub.
- `TeamCode/` – Conține logica personalizată a echipei, pentru modurile Autonom și TeleOp.
- `MeepMeepTesting/` – Simulări de traiectorii autonome folosind MeepMeep.
- `libs/` – Biblioteci externe incluse.
- `gradle/` – Configurații de build.

---

## 🔍 Declarație privind componentele externe

Conform regulamentului concursului, declarăm că următoarele componente **nu au fost realizate integral de autori**, ci sunt utilizate cu respectarea licențelor oficiale:

- 📦 **FTC SDK** – Platforma oficială de dezvoltare, furnizată de FIRST și REV Robotics.
- 📦 **Road Runner v1.0** – Bibliotecă open-source pentru traiectorii autonome, dezvoltată de Acme Robotics.
- 📦 **MeepMeep** – Aplicație desktop pentru simularea traiectoriilor Road Runner.
- 🖼️ **Elemente grafice și media oficiale FTC** – utilizate strict pentru scopuri educaționale și competiționale.

📌 Toate celelalte fișiere, clase, metode, scheme logice, idei de implementare și comentarii din `TeamCode/`, `MeepMeepTesting/` și `HardwareMap` sunt create de membrii echipei InfinityBolts #19093.

---

## 👥 Componența echipei și roluri

Proiectul a fost realizat de o echipă formată din 2 elevi, fiecare având responsabilități bine definite:

### 1. **Danci Iacob Valentin**
- Rol principal: dezvoltarea modului **TeleOp**
- Contribuții:
   - Implementarea logicii pentru controlere și sistemul mecanum drive.
   - Crearea și integrarea funcțiilor de acțiune ale robotului în control manual.
   - Adaptarea comenzilor pentru un răspuns fluid în timpul competiției.

### 2. **Zlagnean Nicolaie Ștefan**
- Rol principal: dezvoltarea modului **Autonom**
- Contribuții:
   - Crearea traiectoriilor autonome în MeepMeep și integrarea acestora în Road Runner.
   - Calibrarea odometriilor pentru o acuratețe sporită în detectarea poziției.
   - Ajustări ale poziției inițiale și aliniere automată pentru execuție optimă în meci.

---

## 🧠 Arhitectură software

- **HardwareMap personalizat** – Inițializare dedicată pentru toate componentele robotului (motoare, encodere, servouri).
- **Moduri Autonome multiple** – Adaptate în funcție de poziția pe teren și detectarea obiectelor.
- **Mecanum Drive + controlere dinamice** – Sistem de deplasare omnidirecțional, cu suport pentru precizie și rotație în moduri TeleOp.
- **Simulări în MeepMeep** – Vizualizarea și testarea traiectoriilor autonome înainte de implementarea pe robotul fizic.

---

## ⚙️ Cerințe pentru rulare

- **Java JDK 8+**
- **Android Studio** (recomandat Arctic Fox sau mai nou)
- **Dispozitiv FTC Control Hub / Expansion Hub**
- **MeepMeep** (opțional pentru testare autonomă)

---

## 🔧 Instrucțiuni de utilizare

1. Clonează repository-ul:
   ```bash
   git clone https://github.com/InfinityBolts-FTC19093/IntoTheDeep_final.git
   ```

2. Deschide-l în Android Studio și lasă build-ul să se finalizeze automat.

3. Conectează-te la robot (Control Hub) și încarcă aplicația folosind butonul “Run”.

---

## 📄 Licență

Acest proiect este licențiat sub **BSD-3-Clause-Clear**, compatibil cu utilizarea în scop educațional și competițional. Pentru detalii complete, consultă fișierul `LICENSE`.

---

## 🏁 Concluzie

Proiectul IntoTheDeep_final reflectă munca, dedicația și abilitățile tehnice ale echipei InfinityBolts FTC #19093 în sezonul **INTO THE DEEP (2024-2025)**. Codul și strategiile au fost dezvoltate conform cerințelor FTC și pot fi adaptate pentru viitoare implementări educaționale și competiționale.
