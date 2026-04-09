#pragma once
// ─── IOptimizer ───────────────────────────────────────────────────────────────
// Common ask/tell interface shared by CMA-ES and the future GP-ARD optimizer.
// Both receive normalised [0,1]^n candidates from ask() and fitness values
// (lower = better) via tell().

#include <vector>
#include <random>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>

struct IOptimizer {
    virtual ~IOptimizer() = default;
    virtual void                             setMean(const std::vector<double>& x0) = 0;
    // Returns the next batch of candidate solutions in [0,1]^n space.
    virtual std::vector<std::vector<double>> ask()  = 0;
    // Feed fitness for each candidate from the last ask() (one-to-one correspondence).
    virtual void                             tell(const std::vector<double>& fits) = 0;
    virtual const std::vector<double>&       bestX()      const = 0;
    virtual double                           bestFit()    const = 0;  // lower = better
    virtual bool                             converged()  const = 0;
    virtual int                              lambda()     const = 0;  // batch size
    virtual int                              generation() const = 0;
    virtual double                           sigma()      const { return -1.0; }
};

// ─── CMAES ────────────────────────────────────────────────────────────────────
// Hansen (2016) "The CMA Evolution Strategy: A Tutorial", Algorithm 1.
// Operates in normalised [0,1]^n space; out-of-bounds candidates are valid and
// handled by the objective via clamping in decode().

class CMAES : public IOptimizer {
public:
    explicit CMAES(int n, double sigma0 = 0.3) : n_(n), gen_(0), sigma_(sigma0) {
        // Population and selection sizes
        lambda_ = 4 + (int)(3 * std::log(n));
        mu_     = lambda_ / 2;

        // Recombination weights w_i ∝ ln(μ + 0.5) − ln(i+1), normalised to sum = 1
        double wsum = 0, wsum_sq = 0;
        weights_.resize(mu_);
        for (int i = 0; i < mu_; ++i) {
            weights_[i] = std::log(mu_ + 0.5) - std::log(i + 1.0);
            wsum += weights_[i];
        }
        for (auto& w : weights_) { w /= wsum; wsum_sq += w * w; }
        mueff_ = 1.0 / wsum_sq;  // effective selection mass

        // Step-size control
        cs_   = (mueff_ + 2) / (n + mueff_ + 5);
        ds_   = 1 + 2 * std::max(0.0, std::sqrt((mueff_ - 1) / (n + 1)) - 1) + cs_;
        chin_ = std::sqrt((double)n) * (1.0 - 1.0/(4*n) + 1.0/(21.0*n*n));

        // Covariance matrix adaptation
        cc_  = (4 + mueff_ / n) / (n + 4 + 2 * mueff_ / n);
        c1_  = 2.0 / ((n + 1.3) * (n + 1.3) + mueff_);
        cmu_ = std::min(1.0 - c1_,
               2.0 * (mueff_ - 2 + 1.0/mueff_) / ((n + 2.0)*(n + 2.0) + mueff_));

        // State initialisation
        m_.assign(n, 0.5);          // start at centre of [0,1]^n
        p_sigma_.assign(n, 0.0);
        p_c_.assign(n, 0.0);

        // Covariance and eigensystem: start as identity
        C_.assign(n, std::vector<double>(n, 0.0));
        B_.assign(n, std::vector<double>(n, 0.0));
        D_.assign(n, 1.0);
        for (int i = 0; i < n; ++i) { C_[i][i] = 1.0; B_[i][i] = 1.0; }

        // Recompute eigendecomposition at most every ~10n evaluations
        eigenInterval_ = std::max(1, (int)(1.0 / (10.0 * n * (c1_ + cmu_))));

        bestFit_ = std::numeric_limits<double>::infinity();
        bestX_   = m_;
    }

    void setMean(const std::vector<double>& x0) override {
        m_ = x0; bestX_ = x0;
    }

    // Sample λ candidates: x_k = m + σ · B · diag(D) · z_k, z_k ~ N(0,I)
    std::vector<std::vector<double>> ask() override {
        std::normal_distribution<double> N01;
        samples_.resize(lambda_);
        zs_.resize(lambda_);
        for (int k = 0; k < lambda_; ++k) {
            zs_[k].resize(n_);
            for (int i = 0; i < n_; ++i) zs_[k][i] = N01(rng_);
            samples_[k].resize(n_);
            for (int i = 0; i < n_; ++i) {
                double y = 0;
                for (int j = 0; j < n_; ++j) y += B_[i][j] * D_[j] * zs_[k][j];
                samples_[k][i] = m_[i] + sigma_ * y;
            }
        }
        return samples_;
    }

    void tell(const std::vector<double>& fits) override {
        // Rank by fitness (ascending)
        std::vector<int> rank(lambda_);
        std::iota(rank.begin(), rank.end(), 0);
        std::sort(rank.begin(), rank.end(),
                  [&](int a, int b) { return fits[a] < fits[b]; });

        if (fits[rank[0]] < bestFit_) { bestFit_ = fits[rank[0]]; bestX_ = samples_[rank[0]]; }

        // Weighted mean update
        std::vector<double> m_new(n_, 0.0);
        for (int i = 0; i < mu_; ++i)
            for (int j = 0; j < n_; ++j)
                m_new[j] += weights_[i] * samples_[rank[i]][j];

        // y_w = (m_new − m) / σ  and  z_w = Σ w_i z_{i:λ}
        std::vector<double> y_w(n_, 0.0), z_w(n_, 0.0);
        for (int i = 0; i < mu_; ++i)
            for (int j = 0; j < n_; ++j) {
                y_w[j] += weights_[i] * (samples_[rank[i]][j] - m_[j]) / sigma_;
                z_w[j] += weights_[i] * zs_[rank[i]][j];
            }

        // B · z_w (direction in original space under isotropic covariance)
        std::vector<double> Bz_w(n_, 0.0);
        for (int i = 0; i < n_; ++i)
            for (int j = 0; j < n_; ++j)
                Bz_w[i] += B_[i][j] * z_w[j];

        // ── Step-size control (CSA) ───────────────────────────────────────────
        double k_cs = std::sqrt(cs_ * (2 - cs_) * mueff_);
        for (int j = 0; j < n_; ++j)
            p_sigma_[j] = (1 - cs_) * p_sigma_[j] + k_cs * Bz_w[j];

        double ps_norm = 0;
        for (auto v : p_sigma_) ps_norm += v * v;
        ps_norm = std::sqrt(ps_norm);
        sigma_ *= std::exp(cs_ / ds_ * (ps_norm / chin_ - 1));

        // Heaviside h_σ: suppress p_c rank-one update when step size is large
        double ps_thresh = (1.4 + 2.0/(n_+1)) * chin_;
        double ps_eff    = ps_norm / std::sqrt(1 - std::pow(1-cs_, 2.0*(gen_+1)));
        int h_sigma      = (ps_eff < ps_thresh) ? 1 : 0;

        // ── Covariance matrix adaptation (CMA) ───────────────────────────────
        double k_cc = std::sqrt(cc_ * (2 - cc_) * mueff_);
        for (int j = 0; j < n_; ++j)
            p_c_[j] = (1 - cc_) * p_c_[j] + h_sigma * k_cc * y_w[j];

        double delta_h = (1 - h_sigma) * cc_ * (2 - cc_);
        for (int i = 0; i < n_; ++i) {
            for (int j = 0; j <= i; ++j) {
                // Rank-one + rank-μ updates
                C_[i][j] = (1 - c1_ - cmu_) * C_[i][j]
                         + c1_ * (p_c_[i]*p_c_[j] + delta_h * C_[i][j]);
                for (int k = 0; k < mu_; ++k) {
                    double yi = (samples_[rank[k]][i] - m_[i]) / sigma_;
                    double yj = (samples_[rank[k]][j] - m_[j]) / sigma_;
                    C_[i][j] += cmu_ * weights_[k] * yi * yj;
                }
                C_[j][i] = C_[i][j];  // enforce symmetry
            }
        }

        if (gen_ % eigenInterval_ == 0) updateEigen();

        m_ = m_new;
        ++gen_;
    }

    const std::vector<double>& bestX()      const override { return bestX_; }
    double                     bestFit()    const override { return bestFit_; }
    bool                       converged()  const override { return sigma_ < 1e-8; }
    int                        lambda()     const override { return lambda_; }
    int                        generation() const override { return gen_; }
    double                     sigma()      const override { return sigma_; }

private:
    // Eigen-decompose C (symmetric PD) → C = B · diag(D²) · Bᵀ.
    // cv::eigen returns eigenvalues descending; eigenvecs.row(i) is the i-th eigenvector.
    // We store B with columns = eigenvectors: B_[row][col] = evecs(col, row).
    void updateEigen() {
        cv::Mat Cmat(n_, n_, CV_64F);
        for (int i = 0; i < n_; ++i)
            for (int j = 0; j < n_; ++j)
                Cmat.at<double>(i, j) = C_[i][j];
        cv::Mat evals, evecs;
        cv::eigen(Cmat, evals, evecs);
        for (int i = 0; i < n_; ++i) {
            D_[i] = std::sqrt(std::max(evals.at<double>(i), 1e-20));
            for (int j = 0; j < n_; ++j)
                B_[j][i] = evecs.at<double>(i, j);
        }
    }

    int    n_, lambda_, mu_, gen_, eigenInterval_;
    double sigma_, mueff_, cs_, ds_, chin_, cc_, c1_, cmu_;

    std::vector<double>              weights_, m_, p_sigma_, p_c_, D_;
    std::vector<std::vector<double>> C_, B_;           // covariance + eigenvectors
    std::vector<std::vector<double>> samples_, zs_;    // last ask() output + z-samples

    std::vector<double> bestX_;
    double              bestFit_;
    std::mt19937        rng_{std::random_device{}()};
};
