#ifndef GMM_H
#define GMM_H
#include <Eigen/Core>
#include <math.h>

using namespace std;

// Define the number of categories
#define GMM_COMPONENTS 3
#define GMM_MAX_ITER 30
#define GMM_THRESHOLD 0.01f
typedef Eigen::Array<float, GMM_COMPONENTS, 1> GMM_Array;

// Gaussian mixture model for one-dimensional features
class GaussianMixture
{
public:
    GaussianMixture(){};
    GMM_Array weights_, means_, covariances_;
    bool converged_ = false;

    // Initialize distribution using kmeans
    void kmeans_init(vector<float> &X)
    {
        int n_components = GMM_COMPONENTS;
        int n_samples = X.size();
        sort(X.begin(), X.end());
        Eigen::Array<float, Eigen::Dynamic, 1> X_s;
        X_s = Eigen::Map<Eigen::Array<float, Eigen::Dynamic, 1>>(X.data(), n_samples, 1);

        // KMeans++ initialize Cluster Center
        means_(0) = X_s(0);
        for (int j = 1; j < n_components - 1; j++)
        {
            means_(j) = X_s(int(j * n_samples / (n_components - 1)));
        }
        means_(n_components - 1) = X_s(n_samples - 1);

        GMM_Array last_means = GMM_Array::Zero();

        for (int iter = 0; iter < GMM_MAX_ITER; iter++)
        {
            Eigen::Array<float, Eigen::Dynamic, 3> dist;
            dist.resize(n_samples, 3);
            // Calculate the distance from all points to different cluster centers
            for (int j = 0; j < n_components; j++)
            {
                dist.col(j) = (X_s - means_(j)).abs();
            }
            // Reassign points
            vector<vector<float>> clusters;
            clusters.resize(n_components);
            for (int i = 0; i < n_samples; i++)
            {
                Eigen::Index idx0, idx1;
                dist.row(i).minCoeff(&idx0, &idx1);
                clusters[idx1].emplace_back(X_s(i));
            }
            for (int j = 0; j < n_components; j++)
            {
                means_(j) = accumulate(clusters[j].begin(), clusters[j].end(), 0.0) / clusters[j].size();
            }

            if ((means_ - last_means).matrix().norm() < GMM_THRESHOLD || iter >= GMM_MAX_ITER - 1)
            {
                covariances_ = GMM_Array::Zero();
                // Calculate weights and variances
                for (int j = 0; j < n_components; j++)
                {
                    weights_(j) = float(clusters[j].size()) / float(n_samples);
                    for (int si = 0; si < clusters[j].size(); si++)
                    {
                        covariances_(j) += pow(clusters[j][si] - means_(j), 2);
                    }
                    covariances_(j) = covariances_(j) / clusters[j].size();

                    // To prevent the occurrence of nan
                    if (weights_(j) == 0 || isnan(means_(j)) || isnan(covariances_(j)))
                        weights_(j) = means_(j) = covariances_(j) = 0;
                }
                break;
            }
            last_means = means_;
        }
    };

    void fit(vector<float> &X)
    {
        // Initialize weights, mean, and variance
        kmeans_init(X);

        // cout << "weights_:" << weights_ << endl;
        // cout << "means_:" << means_ << endl;
        // cout << "covariances_:" << covariances_ << endl;
        // cout << "*********************" << endl;

        int n_components = GMM_COMPONENTS;
        int n_samples = X.size();

        // Responsiveness
        Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic> gamma;
        gamma.resize(n_samples, n_components);
        converged_ = false;
        GMM_Array last_weights = GMM_Array::Zero();

        for (int iter = 0; iter < GMM_MAX_ITER; iter++)
        {
            // E-step
            for (int i = 0; i < n_samples; i++)
            {
                for (int j = 0; j < n_components; j++)
                {
                    if (weights_(j) == 0)
                    {
                        gamma(i, j) = 0;
                        continue;
                    }
                    if (covariances_(j) == 0)
                    {
                        if (X[i] == means_(j))
                            gamma(i, j) = weights_(j);
                        else
                            gamma(i, j) = 0;
                        continue;
                    }
                    gamma(i, j) = weights_(j) * exp(-0.5f * pow(X[i] - means_(j), 2) / covariances_(j)) / sqrt(2.0f * M_PI * covariances_.abs()(j));
                }
                gamma.row(i) = gamma.row(i) / gamma.row(i).sum();
            }

            // M-step
            for (int j = 0; j < n_components; j++)
            {
                float means_tmp = 0.0, covariances_tmp = 0.0;
                float gamma_components = gamma.col(j).sum();

                for (int i = 0; i < n_samples; i++)
                {
                    means_tmp += gamma(i, j) * X[i];
                    covariances_tmp += gamma(i, j) * pow(X[i] - means_(j), 2);
                }
                means_(j) = means_tmp / gamma_components;
                covariances_(j) = covariances_tmp / gamma_components;
                weights_(j) = gamma_components / n_samples;
            }

            if ((weights_ - last_weights).matrix().norm() < GMM_THRESHOLD || iter >= GMM_MAX_ITER - 1)
            {
                converged_ = true;
                for (int j = 0; j < n_components; j++)
                {
                    // To prevent the occurrence of nan
                    if (weights_(j) == 0 || isnan(means_(j)) || isnan(covariances_(j)))
                        weights_(j) = means_(j) = covariances_(j) = 0;
                }
                break;
            }
            last_weights = weights_;
        }
    };

    // calculate the KL divergence between GMMs using the method in paper "APPROXIMATING THE KULLBACK LEIBLER DIVERGENCE BETWEEN GAUSSIAN MIXTURE MODELS"Chapter6.
    float gmm_kl_divergence(GaussianMixture &GM2)
    {
        int n_components = GMM_COMPONENTS;
        GMM_Array Dgold;
        bool Dgold_vaild = false;
        // Calculate the KL divergence from GM_tar to this
        for (int j1 = 0; j1 < n_components; j1++)
        {
            if (weights_(j1) == 0)
            {
                Dgold(j1) = 0;
                continue;
            }

            float m_min = 1000;
            int m_min_idx = -1;
            for (int j2 = 0; j2 < n_components; j2++)
            {
                if (GM2.weights_(j2) == 0)
                    continue;
                float ma = gaussian_kl_divergence(means_(j1), covariances_(j1), GM2.means_(j2), GM2.covariances_(j2)) - log(GM2.weights_(j2));
                if (ma < m_min)
                {
                    m_min = ma;
                    m_min_idx = j2;
                }
            }
            if (m_min_idx > -1)
            {
                Dgold(j1) = weights_(j1) * (gaussian_kl_divergence(means_(j1), covariances_(j1), GM2.means_(m_min_idx), GM2.covariances_(m_min_idx)) + log(weights_(j1) / GM2.weights_(m_min_idx)));
                Dgold_vaild = true;
            }
            else
            {
                Dgold(j1) = 0;
            }
        }
        if (Dgold_vaild)
            return Dgold.sum();
        else
            return -1;
    };

    // Calculate the KL divergence of two univariate Gaussian distributions
    inline float gaussian_kl_divergence(float mean1, float var1, float mean2, float var2)
    {
        return log(var2 / var1) + (pow(var1, 2) + pow(mean1 - mean2, 2)) / (2 * pow(var2, 2)) - 0.5;
    };
};

#endif