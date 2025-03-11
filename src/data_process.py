import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
import pywt
from scipy.fftpack import fft
from scipy.stats import moment

# Obtener el último archivo de datos en emg_data
emg_data_dir = os.path.join(os.path.dirname(__file__), "emg_data")
latest_file = max([os.path.join(emg_data_dir, f) for f in os.listdir(emg_data_dir) if f.endswith(".csv")], key=os.path.getctime)
os_file_path = latest_file

# Cargar datos de archivo CSV usando pandas
df = pd.read_csv(os_file_path)

# Asegurarse de que la columna 'EMG_Raw' no tenga valores faltantes
data = df['EMG_Raw'].dropna().to_numpy()
data = data[data != 0]  # Eliminar valores 0

# Parámetros del filtro Laguerre
def laguerre_filter(signal, alpha=0.9, order=5):
    N = len(signal)
    L = np.zeros((order, N))
    L[0, :] = signal
    for k in range(1, order):
        L[k, :] = alpha * (L[k - 1, :] - np.roll(L[k - 1, :], 1))
        L[k, 0] = 0
    return np.sum(L, axis=0)

# Filtrado con Meyer
wavelet = 'dmey'
coeffs = pywt.wavedec(data, wavelet, level=5)
threshold = np.median(np.abs(coeffs[-1])) / 0.6745
filtered_coeffs = [np.sign(c) * np.maximum(np.abs(c) - threshold, 0) if i > 0 else c for i, c in enumerate(coeffs)]
filtered_data = pywt.waverec(filtered_coeffs, wavelet)

# Filtrado con Laguerre
filtered_data = laguerre_filter(filtered_data, alpha=0.9, order=5)


# 🔹 CÁLCULO DE MÉTRICAS EN DOMINIO TEMPORAL 🔹
emg_integrated = np.sum(np.abs(filtered_data))  # EMG integrado
mav = np.mean(np.abs(filtered_data))  # Valor Medio Absoluto (MAV)
integral_cuadrada = np.sum(filtered_data ** 2)  # Integral Cuadrada Simple
varianza = np.var(filtered_data)  # Varianza
momento_3 = np.abs(moment(filtered_data, moment=3))  # Tercer Momento Temporal
momento_4 = np.abs(moment(filtered_data, moment=4))  # Cuarto Momento Temporal
momento_5 = np.abs(moment(filtered_data, moment=5))  # Quinto Momento Temporal


fs = 1000  # Frecuencia de muestreo
N = len(filtered_data)

fft_result = fft(filtered_data)
frequencies = np.fft.fftfreq(N, d=1/fs)
magnitudes = np.abs(fft_result[:N//2])  # Tomamos solo las frecuencias positivas
frequencies = frequencies[:N//2]

# Cálculo de Frecuencia Media (MNF) y Frecuencia Mediana (MF)
mnf = np.sum(frequencies * magnitudes) / np.sum(magnitudes)  # Frecuencia Media
cumsum_mag = np.cumsum(magnitudes)
mf = frequencies[np.where(cumsum_mag >= np.sum(magnitudes) / 2)[0][0]]  # Frecuencia Mediana


print("📌 MÉTRICAS EN DOMINIO TEMPORAL:")
print(f"✅ EMG Integrado: {emg_integrated:.4f}")
print(f"✅ MAV: {mav:.4f}")
print(f"✅ Integral Cuadrada Simple: {integral_cuadrada:.4f}")
print(f"✅ Varianza: {varianza:.4f}")
print(f"✅ Momento 3: {momento_3:.4f}")
print(f"✅ Momento 4: {momento_4:.4f}")
print(f"✅ Momento 5: {momento_5:.4f}")

print("\n📌 MÉTRICAS EN DOMINIO DE FRECUENCIA:")
print(f"✅ Frecuencia Media (MNF): {mnf:.4f} Hz")
print(f"✅ Frecuencia Mediana (MF): {mf:.4f} Hz")

plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(filtered_data, label='Filtered Signal (Meyer + Laguerre)')
plt.title('Filtered Signal')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(frequencies, magnitudes, label="FFT Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("Frequency Spectrum")
plt.legend()

plt.tight_layout()
plt.show()