# Unity Playback

`simulate.py unity <scenario> [stride]` komutu Unity icin hazir JSON uretir.

Ornek:

```bash
python3 simulate.py unity 3
python3 simulate.py unity 3 5
```

Uretilen dosya:

`logs/unity/s3.json`

Koordinat donusumu:

- Simulasyon: `NED = (north, east, down)`
- Unity: `x = east`, `y = up`, `z = north`

Unity kurulumu:

1. `unity/SimulationPlayback.cs` dosyasini Unity projesine ekle.
2. Uretilen JSON dosyasini Unity'ye `TextAsset` olarak import et.
3. Bos bir GameObject uzerine `SimulationPlayback` component'ini ekle.
4. `jsonFile`, `missileTransform`, `targetTransform` alanlarini Inspector'dan bagla.
5. Play'e basinca fuze ve hedef JSON zaman serisine gore hareket eder.

Not:

- `stride=5` gibi bir deger verirsen daha hafif bir JSON elde edersin.
- Varsayilan olcekte `1 Unity birimi = 1 metre`.
