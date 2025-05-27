//
// Created by Zhaohong Liu on 24-10-15.
//

#ifndef MOGENLIB_CONVERTOR_H
#define MOGENLIB_CONVERTOR_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>

class Convertor {
public:
    static void q2EulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);

    static void q2EulerAngle(const Eigen::Quaterniond& q, Eigen::Vector3d& euler);

    static void euler2Quaternion(Eigen::Quaterniond& q, const Eigen::Vector3d& euler);

    static Eigen::Quaterniond euler2Quaternion(const Eigen::Vector3d& euler);

    /**
     * Fast pseudoinverse based on full rank cholesky factorisation
     * Courrieu, P. (2008). Fast Computation of Moore-Penrose Inverse Matrices, 8(2), 25–29.
     * http://arxiv.org/abs/0804.4809
     */
    template <typename T, int Rows, int Cols>
    static bool getInv(const Eigen::Matrix<T, Rows, Cols> &G, Eigen::Matrix<T, Cols, Rows> &res) {
        // ref PX4-Autopilot src/lib/matrix, gen by chatgpt-4o
        if constexpr (Rows >= Cols) {
            Eigen::Matrix<T, Cols, Cols> A = G.transpose() * G;  // Square matrix
            Eigen::LDLT<Eigen::Matrix<T, Cols, Cols>> ldlt(A);   // LDLT decomposition for stability
            if (ldlt.info() != Eigen::Success) {
                return false;  // Matrix is singular
            }
            res = ldlt.solve(G.transpose());
        } else {
            Eigen::Matrix<T, Rows, Rows> A = G * G.transpose();  // Square matrix
            Eigen::LDLT<Eigen::Matrix<T, Rows, Rows>> ldlt(A);   // LDLT decomposition for stability
            if (ldlt.info() != Eigen::Success) {
                return false;  // Matrix is singular
            }
            res = G.transpose() * ldlt.solve(Eigen::Matrix<T, Rows, Rows>::Identity(A.rows(), A.cols()));
        }
        return true;
    }

    template <typename T, int Rows, int Cols>
    static void printMatrix(const Eigen::Matrix<T, Rows, Cols> &G) {
        std::cout << std::fixed << std::setprecision(4);

        for (int row = 0; row < G.rows(); ++row) {
            std::cout << "| ";
            for (int col = 0; col < G.cols(); ++col) {
                std::cout << std::setw(8) << G(row, col) << " ";
            }
            std::cout << "|\n";
        }

        std::cout << std::endl;
    }
};


#endif //MOGENLIB_CONVERTOR_H
