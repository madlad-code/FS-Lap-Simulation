# FS Lap Sim - Varvtidssimulering för Formula Student

En kraftfull motor för att simulera varvtider för Formula Student-bilar, utvecklad för att optimera fordonsdesign genom data-drivna beslut.

## Mål med projektet
Huvudmålet var att skapa ett verktyg som snabbt kan utvärdera hur olika designval (t.ex. vikt, effekt, downforce) påverkar varvtiden på specifika banor. Genom att använda en "quasi-steady-state"-modell kan simuleringen köras på bråkdelen av en sekund, vilket möjliggör omfattande känslighetsanalyser och optimeringsloopar under designfasen.

## Kompetenser som testas
- **Fordonsdynamik:** Implementering av Pacejka-däckmodeller, aerodynamisk downforce och longitudinell/lateral greppkoppling (g-g envelope).
- **Fysiksimulering:** Numerisk integration av rörelseekvationer för att beräkna hastighetsprofiler längs en godtycklig bana.
- **Python & Prestanda:** Effektiv användning av `NumPy` för att hantera stora datamängder (telemetri) och snabba beräkningar.
- **Ingenjörsmetodik:** Utveckling av känslighetsanalyser (`sensitivity.py`) för att rangordna tekniska parametrar efter deras faktiska inverkan på prestanda.

## Moduler
- **`vehicle.py`**: Definition av fordonsparametrar och däckmodell.
- **`simulator.py`**: Huvudmotor för simulering (forward/backward integration).
- **`sensitivity.py`**: Verktyg för parametriska svep och rangordning av variabler.

## Installation & Användning
Kräver Python och NumPy.
```bash
pip install numpy
python src/simulator.py --track fsg
python src/sensitivity.py --sweep all
```

## Metod
1. Parsar bangeometri från CSV.
2. Beräknar krökning (`κ`) och maxhastighet (`v_max`) baserat på greppgränser.
3. Kör en "forward pass" (acceleration) och en "backward pass" (bromsning).
4. Integrerar `dt = ds / v` för att få total varvtid och telemetri.

## Framtida förbättringar & Reflektion
Detta projekt har gett mig en djupare förståelse för hur små designval (som t.ex. 10 kg viktminskning vs 0.2 ökning i Cl) faktiskt påverkar prestandan på banan. Nästa steg för simulatorn är:
- **Avancerade däckmodeller:** Implementera Pacejka 5.2 för att bättre simulera kombinerat grepp (lateral + longitudinal).
- **Drivlinemodellering:** Lägga till specifika vridmomentkurvor för elmotorer och termisk simulering av batteriet.
- **3D-Bana:** Hantera höjdskillnader (gradienter) för att simulera banor som t.ex. Red Bull Ring mer exakt.

---
Oscar Enghag · Datateknik LTH
