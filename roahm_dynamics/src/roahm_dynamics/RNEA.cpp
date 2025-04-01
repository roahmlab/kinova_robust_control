#include "roahm_dynamics/RNEA.hpp"

namespace Roahm
{
namespace Dynamics
{

MultiBodyDynamics::MultiBodyDynamics() {
    S.resize(NB);
    Xup.resize(NB);
    v.resize(NB);
    v_aux.resize(NB);
    a.resize(NB);
    K.resize(NB);
    Yfull.resize(6 * NB, 10 * NB);
    Y.resize(NB, 10 * NB);
    tau.resize(NB);
    tau_inf.resize(NB);
    tau_sup.resize(NB);
}

MultiBodyDynamics::MultiBodyDynamics(const std::shared_ptr<Model::model>& modelPtr_in) :
    modelPtr_(modelPtr_in) {
    NB = modelPtr_->NB;
    S.resize(NB);
    Xup.resize(NB);
    v.resize(NB);
    v_aux.resize(NB);
    a.resize(NB);
    K.resize(NB);
    Yfull.resize(6 * NB, 10 * NB);
    Y.resize(NB, 10 * NB);
    tau.resize(NB);
    tau_inf.resize(NB);
    tau_sup.resize(NB);
}

MultiBodyDynamics::MultiBodyDynamics(const std::shared_ptr<Model::model>& modelPtr_in, 
                                     const double phi_eps_in) :
    modelPtr_(modelPtr_in) {
    NB = modelPtr_->NB;
    S.resize(NB);
    Xup.resize(NB);
    v.resize(NB);
    v_aux.resize(NB);
    a.resize(NB);
    K.resize(NB);
    Yfull.resize(6 * NB, 10 * NB);
    Y.resize(NB, 10 * NB);
    tau.resize(NB);
    tau_inf.resize(NB);
    tau_sup.resize(NB);
}

void MultiBodyDynamics::Yphi_passive(const VecX& q, 
                                     const VecX& q_d, 
                                     const VecX& q_aux_d,
                                     const VecX& q_dd,
                                     const bool add_gravity) {
    if (q.size() != NB || 
        q_d.size() != NB || 
        q_aux_d.size() != NB || 
        q_dd.size() != NB) {
        throw std::invalid_argument("Yphi_passive: input vector size mismatch!");
    }

    if (Utils::ifTwoVectorEqual(q, q_copy, 1e-16) && 
        Utils::ifTwoVectorEqual(q_d, q_d_copy, 1e-16) && 
        Utils::ifTwoVectorEqual(q_aux_d, q_aux_d_copy, 1e-16) && 
        Utils::ifTwoVectorEqual(q_dd, q_dd_copy, 1e-16)) {
        return;
    }

    Y.setZero();
    Yfull.setZero();

    // Forward pass
    for (int i = 0; i < NB; i++) {
        Spatial::jcalc(Xj, S[i], modelPtr_->jtype[i], q[i]);

        vJ = S[i] * q_d[i];
        vJ_aux = S[i] * q_aux_d[i];
        Xup[i] = Xj * modelPtr_->Xtree[i];

        if (modelPtr_->parent[i] == -1) {
            v[i] = vJ;
            v_aux[i] = vJ_aux;
            if (add_gravity) {
                a[i] = Xup[i] * (-modelPtr_->a_grav) + S[i] * q_dd[i];
            }
            else {
                a[i] = S[i] * q_dd[i];
            }
        }
        else {
            v[i] = Xup[i] * v[modelPtr_->parent[i]] + vJ;
            v_aux[i] = Xup[i] * v_aux[modelPtr_->parent[i]] + vJ_aux;
            a[i] = Xup[i] * a[modelPtr_->parent[i]] + S[i] * q_dd[i] + Spatial::crm(v_aux[i]) * vJ;
        }

        double a1 = a[i](0);
        double a2 = a[i](1);
        double a3 = a[i](2);
        double a4 = a[i](3);
        double a5 = a[i](4);
        double a6 = a[i](5);

        double v1 = v[i](0);
        double v2 = v[i](1);
        double v3 = v[i](2);
        double v4 = v[i](3);
        double v5 = v[i](4);
        double v6 = v[i](5);

        double va1 = v_aux[i](0);
        double va2 = v_aux[i](1);
        double va3 = v_aux[i](2);
        double va4 = v_aux[i](3);
        double va5 = v_aux[i](4);
        double va6 = v_aux[i](5);
        
        K[i] <<      a1,     a2 - v1*va3,     a3 + v1*va2, -v2*va3, v2*va2 - v3*va3,  v3*va2, v5*va2 - v2*va5 - v3*va6 + v6*va3,              a6 + v1*va5 - v4*va2,              v1*va6 - a5 - v4*va3,      v6*va5 - v5*va6,
                 v1*va3,     a1 + v2*va3, v3*va3 - v1*va1,      a2,     a3 - v2*va1, -v3*va1,              v2*va4 - a6 - v5*va1, v4*va1 - v1*va4 - v3*va6 + v6*va3,              a4 + v2*va6 - v5*va3,      v4*va6 - v6*va4,
                -v1*va2, v1*va1 - v2*va2,     a1 - v3*va2,  v2*va1,     a2 + v3*va1,      a3,              a5 + v3*va4 - v6*va1,              v3*va5 - a4 - v6*va2, v4*va1 - v1*va4 - v2*va5 + v5*va2,      v5*va4 - v4*va5,
                      0,               0,               0,       0,               0,       0,                 - v2*va2 - v3*va3,                       v1*va2 - a3,                       a2 + v1*va3, a4 - v5*va3 + v6*va2,
                      0,               0,               0,       0,               0,       0,                       a3 + v2*va1,                 - v1*va1 - v3*va3,                       v2*va3 - a1, a5 + v4*va3 - v6*va1,
                      0,               0,               0,       0,               0,       0,                       v3*va1 - a2,                       a1 + v3*va2,                 - v1*va1 - v2*va2, a6 - v4*va2 + v5*va1;

        Yfull.block(6*i, 10*i, 6, 10) = K[i];
    }

    // Backward pass
    for (int i = NB - 1; i >= 0; i--) {
        if (modelPtr_->parent[i] != -1) {
            Yfull.middleRows(6*modelPtr_->parent[i], 6) += Xup[i].transpose() * Yfull.middleRows(6*i, 6);
        }

        Y.row(i) = S[i].transpose() * Yfull.middleRows(6*i, 6);
    }

    // Copy the evaluated joint positions, velocities, and accelerations
    q_copy = q;
    q_d_copy = q_d;
    q_aux_d_copy = q_aux_d;
    q_dd_copy = q_dd;
}

void MultiBodyDynamics::rnea(const VecX& q, 
                             const VecX& q_d, 
                             const VecX& q_aux_d,
                             const VecX& q_dd,
                             const bool add_gravity) {
    if (q.size() != NB || 
        q_d.size() != NB || 
        q_aux_d.size() != NB || 
        q_dd.size() != NB) {
        throw std::invalid_argument("MultiBodyDynamics::rnea: input vector size mismatch!");
    }

    Yphi_passive(q, q_d, q_aux_d, q_dd, add_gravity);
    tau = Y * modelPtr_->phi;

    // motor dynamics
    for (int i = 0; i < NB; i++) {
        tau(i) += modelPtr_->damping[i] * q_d(i) +
                  modelPtr_->transmissionInertia[i] * q_dd(i) + modelPtr_->offset[i];
                  
        if (fabs(q_d(i)) > FRICTION_APPLY_VELOCITY_THRESHOLD) {
            if (q_d(i) > 0) {
                tau(i) += modelPtr_->friction[i];
            }
            else {
                tau(i) -= modelPtr_->friction[i];
            }
        }
    }
}

void MultiBodyDynamics::rnea_interval(const VecX& q, 
                                      const VecX& q_d, 
                                      const VecX& q_aux_d,
                                      const VecX& q_dd,
                                      const bool add_gravity) {
    if (q.size() != NB || 
        q_d.size() != NB || 
        q_aux_d.size() != NB || 
        q_dd.size() != NB) {
        throw std::invalid_argument("MultiBodyDynamics::rnea_interval: input vector size mismatch!");
    }

    Yphi_passive(q, q_d, q_aux_d, q_dd, add_gravity);
    tau = Y * modelPtr_->phi;

    tau_inf.setZero();
    tau_sup.setZero();

    for (int i = 0; i < NB; i++) {
        for (int j = 0; j < 10 * NB; j++) {
            // simple interval arithmetic here
            if (Y(i, j) > 0) {
                tau_inf(i) += Y(i, j) * modelPtr_->phi_lb(j);
                tau_sup(i) += Y(i, j) * modelPtr_->phi_ub(j);
            }
            else {
                tau_inf(i) += Y(i, j) * modelPtr_->phi_ub(j);
                tau_sup(i) += Y(i, j) * modelPtr_->phi_lb(j);
            }
        }
    }

    // motor dynamics
    for (int i = 0; i < NB; i++) {
        tau(i) += modelPtr_->damping[i] * q_d(i) +
                  modelPtr_->transmissionInertia[i] * q_dd(i) + modelPtr_->offset[i];

        tau_inf(i) += modelPtr_->offset[i];
        tau_sup(i) += modelPtr_->offset[i];

        if (q_d(i) >= 0) {
            tau_inf(i) += modelPtr_->damping[i] * (1 - modelPtr_->damping_eps[i]) * q_d(i);
            tau_sup(i) += modelPtr_->damping[i] * (1 + modelPtr_->damping_eps[i]) * q_d(i);
        }
        else {
            tau_inf(i) += modelPtr_->damping[i] * (1 + modelPtr_->damping_eps[i]) * q_d(i);
            tau_sup(i) += modelPtr_->damping[i] * (1 - modelPtr_->damping_eps[i]) * q_d(i);
        }

        if (q_dd(i) >= 0) {
            tau_inf(i) += modelPtr_->transmissionInertia[i] * (1 - modelPtr_->transmissionInertia_eps[i]) * q_dd(i);
            tau_sup(i) += modelPtr_->transmissionInertia[i] * (1 + modelPtr_->transmissionInertia_eps[i]) * q_dd(i);
        }
        else {
            tau_inf(i) += modelPtr_->transmissionInertia[i] * (1 + modelPtr_->transmissionInertia_eps[i]) * q_dd(i);
            tau_sup(i) += modelPtr_->transmissionInertia[i] * (1 - modelPtr_->transmissionInertia_eps[i]) * q_dd(i);
        }
                  
        if (fabs(q_d(i)) > FRICTION_APPLY_VELOCITY_THRESHOLD) {
            if (q_d(i) > 0) {
                tau(i) += modelPtr_->friction[i];
                tau_inf(i) += modelPtr_->friction[i] * (1 - modelPtr_->friction_eps[i]);
                tau_sup(i) += modelPtr_->friction[i] * (1 + modelPtr_->friction_eps[i]);
            }
            else {
                tau(i) -= modelPtr_->friction[i];
                tau_inf(i) -= modelPtr_->friction[i] * (1 + modelPtr_->friction_eps[i]);
                tau_sup(i) -= modelPtr_->friction[i] * (1 - modelPtr_->friction_eps[i]);
            }
        }
    }
}

} // namespace Dynamics
} // namespace Roahm