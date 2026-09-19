# Microduck Headless Lab

**[Apri la simulazione](https://thinking0things.github.io/microduck-headless-lab/)**

Una sola andatura del Microduck SG90 headless, con direzione controllabile nel browser.
MuJoCo simula gravità, contatti e sei servomotori; Three.js visualizza le mesh CAD.
Il sito non apre porte seriali, non si connette all’ESP32 e non comanda il robot reale.

## Comandi

- **W / ↑**: avanti.
- **S / ↓**: indietro, invertendo la fase della stessa andatura.
- **A / ←** e **D / →**: curva a sinistra/destra durante il passo.
- **Spazio**: pausa della simulazione.
- Su touch o con il mouse, tocca una direzione per mantenerla; usa **Pausa** per fermare la simulazione.
- **Riparti dal centro**: ripristina posizione, orientamento e fase.
- Trascina la vista per orbitare; rotella o gesto di zoom per avvicinarti.

Le direzioni sono relative al robot, non alla telecamera. Le frecce laterali fanno una curva,
non uno spostamento laterale puro. La pausa congela il tempo della simulazione; non è un
controllore di arresto fisico. Quando la pagina perde il focus la simulazione va in pausa.

## Modello e controllo

Il modello iniziale è `microduck9_none_feet_in.xml`: versione headless con i piedi montati verso
l’interno, sulla quale è stata addestrata l’andatura `walk_feetin_mid`. Le mesh sono congelate
dall’archivio compatibile `shell160_pre_tof`, perché la cartella CAD principale è stata modificata
successivamente. Le revisioni CAD v1.1/v1.2 non sono presentate come se fossero questo modello.

Il controllore periodico è portato da `gait9.py`, con comando di posizione a 50 Hz, limiti di
coppia e velocità dei comandi. Lo sterzo varia fino al 10% l’ampiezza relativa dei due fianchi,
senza imporre direttamente posizione o rotazione al corpo libero. La retromarcia percorre la
stessa traiettoria periodica al contrario. Non viene applicata una forza artificiale di guida.

La verifica automatica esegue 12 secondi per avanti, sinistra, destra e indietro nel motore WASM,
controllando che il robot rimanga in piedi e che le direzioni di svolta siano corrette. Si tratta
di un controllo nominale, non di una garanzia di stabilità per qualunque sequenza: sterzate lunghe,
cambi di direzione o differenze numeriche possono portare a una caduta. La retromarcia può deviare.

## Sviluppo locale

Richiede Node.js 22+ e Python 3 per il semplice server statico.

```sh
npm ci
npm run vendor
npm test
npm run serve
```

Apri `http://127.0.0.1:8768`. I file in `dist/` sono già completi e includono le dipendenze;
per visualizzarli basta anche un qualunque server HTTP statico. Nessuna chiave API o backend.

## GitHub Pages

Il ramo `main` contiene sorgenti, test, dati e dipendenze fissate. Il ramo `gh-pages` contiene
il solo contenuto di `dist/`, servito alla radice del sito del progetto.

```sh
git add .
git commit -m "Update simulation"
git push origin main
git subtree split --prefix dist -b pages-release
git push origin pages-release:gh-pages
git branch -D pages-release
```

In Settings → Pages, la sorgente è il ramo `gh-pages`, cartella `/`.
Gli import e gli asset usano percorsi relativi compatibili con l’URL del progetto.

## Provenienza e licenze di terze parti

- Motore: [Google DeepMind MuJoCo](https://github.com/google-deepmind/mujoco), pacchetto ufficiale
  `@mujoco/mujoco` **3.6.0**, Apache-2.0; licenza in `dist/vendor/mujoco/LICENSE`.
- Visualizzazione: [Three.js](https://github.com/mrdoob/three.js) **0.178.0**, MIT;
  licenza in `dist/vendor/three/LICENSE`.
- Geometria SG90 e controllore: progetto Microduck locale dell’autore. La ricostruzione CAD prende
  come riferimento le proporzioni cinematiche di [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck);
  le mesh upstream non sono incluse in questa repo.

I dati di provenienza del modello sono in `dist/assets/provenance.json`.
