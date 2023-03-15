#include <iostream>
#include <armadillo>
#include "turtlelib/circle_fitting.hpp"
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{

    Circle circle_fitting(std::vector<turtlelib::Vector2D> cluster)
    {
        /*
            Step 1
        */
        auto centroid_x_hat = 0.0;
        auto centroid_y_hat = 0.0;
        auto n_num = 0.0; // Total number of elements

        for (size_t i = 0; i < cluster.size(); i++)
        {
            centroid_x_hat += cluster.at(i).x;
            centroid_y_hat += cluster.at(i).y;
            n_num+=1.0;
        }

        centroid_x_hat/=n_num; // eq...(1)
        centroid_y_hat/=n_num; // eq...(2)

        /*
            Step 2: x_hat -> x & y_hat -> y
        */
        auto x_i = 0.0;
        auto y_i = 0.0;
        auto z_i = 0.0;
        /*
            Step 4
        */
        auto z_mean = 0.0;
        auto n = static_cast<double>(cluster.size());
        /*
            Step 5: Data matrix
        */
        arma::mat Z{};

        for (size_t i = 0; i < cluster.size(); i++)
        {
            x_i = cluster.at(i).x - centroid_x_hat;          // eq...(3)
            y_i = cluster.at(i).y - centroid_y_hat;          // eq...(4)
            z_i = x_i*x_i + y_i*y_i;                         // Step 3

            Z.insert_rows(i,arma::rowvec{z_i, x_i, y_i, 1}); // eq...(6)
            z_mean += z_i;
        }

        /*
            Step 4
        */
        z_mean = z_mean/n;

        /*
            Step 6: Moment matrix
        */
        arma::mat M = (1/n)*trans(Z)*Z;

        /*
            Step 7: Constraint matrix
        */
        arma::mat H(4, 4, arma::fill::zeros);
        H(0,0) = 8.0*z_mean;    // eq...(7)
        H(3,0) = 2.0;
        H(0,3) = 2.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0;

        /*
            Step 9: Singular Value Decomposition of Z
        */
        arma::mat U{};
        arma::vec s{};
        arma::mat V{};
        arma::svd(U, s, V, Z);    // eq...(9)

        arma::colvec A{};
        if (s(3) < 10e-12)
        {
            /*
                Step 10: sigma4 < 10^-12
            */
            A = V.col(3);
        }
        else
        {
            /*
                Step 11: sigma4 > 10^-12
            */
            arma::mat Y = V*arma::diagmat(s)*trans(V);    // eq...(10)
            /*
                Step 8: Constraint matrix inverse
            */
            arma::mat Q = Y*H.i()*Y;
            // eig vec & val Q
            arma::cx_vec eigval{};
            arma::cx_mat eigvec{};
            arma::eig_gen(eigval, eigvec, Q);
            // Get real
            arma::mat real_eigval = arma::real(eigval);
            arma::mat real_eigvec = arma::real(eigvec);
            // Set A* = smallest eig value's vec
            auto min = 1000000.0;
            size_t min_index = 0;

            for (size_t i = 0; i < real_eigval.size(); i++)
            {
                if (real_eigval(i) < min && real_eigval(i) > 0.0) // Find the smallest positive eigval
                {
                    min = real_eigval(i);
                    min_index = i;
                }
            }
            // Extract eigvec that corresponds with the eigen value
            arma::vec A_star = real_eigvec.col(min_index);
            A = Y.i()*A_star;
        }

        /*
            Step 12: Equation for circle eq...(11)
        */
        double a = -A(1)/(2.0*A(0));    // eq...(12)
        double b = -A(2)/(2.0*A(0));    // eq...(13)
        double R = std::sqrt((A(1)*A(1) + A(2)*A(2) - 4.0*A(0)*A(3)) / (4.0*A(0)*A(0))); // eq...(14)

        /*
            Step 13: Get centroid coordinates
        */
        double cx = a + centroid_x_hat;
        double cy = b + centroid_y_hat;

    return {cx, cy, R};
    }
}
