#pragma once
#include <math.h>
#include <string.h>
#include "config.h"

// Second-order IIR section, transposed direct form II.
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float s1 = 0, s2 = 0;

  float step(float x) {
    float y = b0 * x + s1;
    s1 = b1 * x - a1 * y + s2;
    s2 = b2 * x - a2 * y;
    return y;
  }
  void reset() { s1 = s2 = 0; }
  // Set the state as if a constant x0 had been applied forever (no start-up transient).
  void prime(float x0) {
    float y0 = x0 * (b0 + b1 + b2) / (1.0f + a1 + a2);
    s2 = b2 * x0 - a2 * y0;
    s1 = y0 - b0 * x0;
  }
  // RBJ cookbook Butterworth (Q = 1/sqrt 2)
  void designLowpass(float f0, float fs) { design(f0, fs, false); }
  void designHighpass(float f0, float fs) { design(f0, fs, true); }

 private:
  void design(float f0, float fs, bool high) {
    const float q = 0.70710678f;
    float w0 = 2.0f * (float)M_PI * f0 / fs;
    float c = cosf(w0), alpha = sinf(w0) / (2.0f * q);
    float a0 = 1.0f + alpha;
    if (high) {
      b0 = (1.0f + c) / 2.0f / a0;
      b1 = -(1.0f + c) / a0;
      b2 = b0;
    } else {
      b0 = (1.0f - c) / 2.0f / a0;
      b1 = (1.0f - c) / a0;
      b2 = b0;
    }
    a1 = -2.0f * c / a0;
    a2 = (1.0f - alpha) / a0;
  }
};

// Removes single-sample glitches: the median of the last three samples replaces a sample only when
// it is a gross outlier (more than MEDIAN_GLITCH_G from that median). A plain median of 3 is
// nonlinear: on a strong tone near the 400 Hz Nyquist (the buzzer, aliased) it rectifies the tone
// into false low-frequency "shaking", which tripped and then held the alarm. Real I2C glitches are
// many g; real vibration and buzzer pickup stay well below that and pass through untouched.
struct Median3 {
  float a = 0, b = 0;
  bool primed = false;
  float step(float x) {
    if (!primed) {
      a = b = x;
      primed = true;
    }
    float m = fmaxf(fminf(a, b), fminf(fmaxf(a, b), x));
    a = b;
    b = x;
    return fabsf(x - m) > MEDIAN_GLITCH_G ? m : x;
  }
};

// Windowed-sinc low-pass FIR that decimates 3 axes by DECIM.
struct FirDecimator {
  float h[FIR_TAPS];
  float buf[3][FIR_TAPS];
  int idx = 0, phase = 0;
  bool primed = false;

  void design(float fcHz, float fs) {
    const int M = FIR_TAPS - 1;
    float fc = fcHz / fs, sum = 0;
    for (int i = 0; i < FIR_TAPS; i++) {
      float n = i - M / 2.0f;
      float s = (n == 0) ? 2.0f * fc : sinf(2.0f * (float)M_PI * fc * n) / ((float)M_PI * n);
      float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / M);  // Hamming
      h[i] = s * w;
      sum += h[i];
    }
    for (int i = 0; i < FIR_TAPS; i++) h[i] /= sum;  // unity DC gain
  }
  // Returns true when a decimated output is ready in out[3].
  bool push(const float in[3], float out[3]) {
    if (!primed) {
      for (int k = 0; k < 3; k++)
        for (int i = 0; i < FIR_TAPS; i++) buf[k][i] = in[k];
      primed = true;
    }
    for (int k = 0; k < 3; k++) buf[k][idx] = in[k];
    idx = (idx + 1) % FIR_TAPS;
    if (++phase < DECIM) return false;
    phase = 0;
    for (int k = 0; k < 3; k++) {
      float acc = 0;
      int j = idx;  // oldest sample
      for (int i = 0; i < FIR_TAPS; i++) {
        acc += h[i] * buf[k][j];
        if (++j == FIR_TAPS) j = 0;
      }
      out[k] = acc;
    }
    return true;
  }
};

struct Features {
  bool valid = false;
  float rms = 0;          // g, 3-axis filtered, 0.5 s window
  float peak = 0;         // g, max filtered vector magnitude in the window
  float crest = 0;        // peak / rms (sine ~1.4, impulse > 3)
  float decay = 1;        // rms of newest 0.25 s / previous 0.25 s
  float freq = 0;         // Hz, dominant axis, zero-crossing estimate (0 = none)
  float freqQ = 0;        // 0..1, consistency of the measured periods
  float periodicity = 0;  // autocorrelation at one period, 0..1
  uint8_t domAxis = 0;
};

// Sliding-window features on band-passed data at the decimated rate.
class FeatureTracker {
 public:
  static constexpr int WIN = 100;   // 0.5 s energy window
  static constexpr int HIST = 400;  // 2 s history for frequency
  static constexpr int ACF_N = 200; // 1 s autocorrelation window

  void reset() {
    memset(this, 0, sizeof(*this));
    f_.decay = 1;
  }

  void push(float fx, float fy, float fz, float fs) {
    float e2 = fx * fx + fy * fy + fz * fz;
    e2r_[wi_] = e2;
    er_[wi_] = sqrtf(e2);
    wi_ = (wi_ + 1) % WIN;
    if (nWin_ < WIN) nWin_++;
    sumE2_ += e2;  // running sum, re-derived exactly in recompute()
    h_[0][hi_] = fx;
    h_[1][hi_] = fy;
    h_[2][hi_] = fz;
    hi_ = (hi_ + 1) % HIST;
    if (nHist_ < HIST) nHist_++;
    const float a = 0.01f;
    ema_[0] += a * (fx * fx - ema_[0]);
    ema_[1] += a * (fy * fy - ema_[1]);
    ema_[2] += a * (fz * fz - ema_[2]);
    if (++count_ % 10 == 0) recompute(fs);
  }

  const Features& features() const { return f_; }

 private:
  void recompute(float fs) {
    // energy window (exact sums)
    float sum = 0, peak = 0, sNew = 0, sOld = 0;
    const int half = WIN / 2;
    for (int i = 0; i < nWin_; i++) {
      int j = (wi_ - 1 - i + WIN) % WIN;  // i = 0 is newest
      sum += e2r_[j];
      if (er_[j] > peak) peak = er_[j];
      if (i < half) sNew += e2r_[j];
      else sOld += e2r_[j];
    }
    sumE2_ = sum;
    f_.rms = nWin_ ? sqrtf(sum / nWin_) : 0;
    f_.peak = peak;
    f_.crest = f_.rms > 1e-6f ? peak / f_.rms : 0;
    f_.decay = (nWin_ >= WIN && sOld > 1e-12f) ? fminf(sqrtf(sNew / sOld), 5.0f) : 1.0f;

    // dominant axis
    uint8_t d = 0;
    if (ema_[1] > ema_[d]) d = 1;
    if (ema_[2] > ema_[d]) d = 2;
    f_.domAxis = d;

    // chronological copy of the dominant axis
    int n = nHist_;
    for (int i = 0; i < n; i++) tmp_[i] = h_[d][(hi_ - n + i + HIST) % HIST];

    // hysteresis from the newest 0.5 s, so quiet history before an onset is ignored
    float s2 = 0;
    int m = n < WIN ? n : WIN;
    for (int i = n - m; i < n; i++) s2 += tmp_[i] * tmp_[i];
    float hyst = 0.3f * sqrtf(m ? s2 / m : 0);

    // positive-going zero crossings with sub-sample interpolation
    float cross[64];
    int nc = 0;
    bool armed = false;
    for (int i = 1; i < n && nc < 64; i++) {
      if (tmp_[i - 1] < -hyst) armed = true;
      if (armed && tmp_[i - 1] < 0 && tmp_[i] >= 0) {
        float frac = -tmp_[i - 1] / (tmp_[i] - tmp_[i - 1]);
        cross[nc++] = (i - 1) + frac;
        armed = false;
      }
    }
    f_.freq = 0;
    f_.freqQ = 0;
    if (nc >= 3 && hyst > 1e-4f) {
      float meanP = (cross[nc - 1] - cross[0]) / (nc - 1);
      float var = 0;
      for (int i = 1; i < nc; i++) {
        float dp = (cross[i] - cross[i - 1]) - meanP;
        var += dp * dp;
      }
      float cv = sqrtf(var / (nc - 1)) / meanP;
      f_.freq = fs / meanP;
      f_.freqQ = fmaxf(0.0f, fminf(1.0f, 1.0f - 2.0f * cv));
    }

    // periodicity: normalized autocorrelation at one period over the newest 1 s
    f_.periodicity = 0;
    if (f_.freq > 0) {
      int L = (int)lroundf(fs / f_.freq);
      if (L >= 2 && n >= ACF_N + L) {
        float sxy = 0, sxx = 0, syy = 0;
        for (int i = n - ACF_N; i < n; i++) {
          float x = tmp_[i], y = tmp_[i - L];
          sxy += x * y;
          sxx += x * x;
          syy += y * y;
        }
        if (sxx > 1e-12f && syy > 1e-12f) f_.periodicity = fmaxf(0.0f, sxy / sqrtf(sxx * syy));
      }
    }
  }

  float e2r_[WIN], er_[WIN];
  float h_[3][HIST];
  float tmp_[HIST];
  float ema_[3];
  float sumE2_;
  int wi_, nWin_, hi_, nHist_;
  uint32_t count_;
  Features f_;
};
