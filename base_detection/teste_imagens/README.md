# Pasta de teste

Coloque aqui as fotos que quer testar (`.jpg`/`.png`) e rode, de dentro de
`base_detection/`:

```bash
python test_raw_detector.py --model models/best_ncnn_model
# ou
python test_raw_detector.py --model models/best_w8a32.tflite
```

Requer só `opencv-python`, `numpy` e (`ncnn` ou `ai-edge-litert`, conforme o
`--model` escolhido) -- não precisa de torch/ultralytics pra rodar isso.

Pra cada imagem em `teste_imagens/`, o script:
- roda `RawDetector` (`../raw_detector.py`) e imprime classe + confiança + centroide,
- salva em `teste_imagens/resultados/<nome>_out.jpg` a mesma imagem com um
  X + círculo marcando o centroide detectado (sem caixa, só o ponto).

Detecções consideram só as classes de forma (`shape_hexagon`, `shape_star`,
`shape_triangle`) -- os números são ignorados de propósito, porque cada base
tem um número impresso dentro da própria forma; contar as duas classes
separadamente faria uma única base contar como duas detecções. Veja
`../raw_detector.py` para o porquê e `../model_export/verify_raw_detector.py`
para a checagem que confirma que esse decode bate com o do ultralytics.
