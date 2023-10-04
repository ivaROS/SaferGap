#ifndef KEYHOLE__H
#define KEYHOLE__H

#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include "ortools/linear_solver/linear_solver.h"
#include <matplot/matplot.h>
#include <algorithm>
// #include <tuple>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// #define PI (3.14159265359)
using namespace std;
using namespace operations_research;

namespace sg_mpc_local_planner{
namespace keyhole{

struct KeyholeParam{
    Eigen::VectorXd a;
    Eigen::Vector2d c1, c2, c3, c4, c5;
    double d1, d2, d3, d4, d5;

    KeyholeParam() {};

    KeyholeParam(Eigen::VectorXd a_in, Eigen::Vector2d c1_in, Eigen::Vector2d c2_in, Eigen::Vector2d c3_in, Eigen::Vector2d c4_in, Eigen::Vector2d c5_in,
                double d1_in, double d2_in, double d3_in, double d4_in, double d5_in)
    {
        a = a_in;

        c1 = c1_in;
        c2 = c2_in;
        c3 = c3_in;
        c4 = c4_in;
        c5 = c5_in;

        d1 = d1_in;
        d2 = d2_in;
        d3 = d3_in;
        d4 = d4_in;
        d5 = d5_in;
    }
};

using namespace Eigen;
#define KPI (3.14159265359)

/**
 * @brief Creates a keyhole barrier function for the ego robot given the points
 * xc (center), r (radius of the circle), p0 (lines starting point inside the circle),
 * p1 (end point of the first line), p2 (end point of the second line)
 * 
 */
class Keyhole{
    public:
        /**
         * @brief Default constructor
         */
        Keyhole();

        /**
         * @brief Default deconstructor
         */
        ~Keyhole();

        /**
         * @brief Construct a new Keyhole object with triangle lines
         * 
         * @param xc Center of the ego circle
         * @param r Radius of the ego circle 
         * @param p0 Starting point of both lines; must be striclty inside the the ego circle 
         * @param p1 Ending point of the first line; must be striclty outside the the ego circle 
         * @param p2 Ending point of the second line; must be striclty outside the the ego circle 
         */
        Keyhole(Vector2d xc, double r, Vector2d p0, Vector2d p1, Vector2d p2);

        /**
         * @brief Construct a new Keyhole object with semi-parallel lines
         *
         * @param xc Center of the ego circle
         * @param r Radius of the ego circle
         * @param q1 Starting point for the first line; must be on the circumference of the ego cricle
         * @param q2 Starting point for the second line; must be on the circumference of the ego cricle
         * @param p1 Ending point of the first line; must be striclty outside the the ego circle
         * @param p2 Ending point of the second line; must be striclty outside the the ego circle
         */
        Keyhole(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d p1, Vector2d p2);

        /**
         * @brief Set the parameters of the keyhole with triangle lines.
         * 
         * @details This method is used to reset the parameters of the keyhole with triangle lines after the object is instantiated.
         * 
         * @param xc Center of the ego circle
         * @param r Radius of the ego circle 
         * @param p0 Starting point of both lines; must be striclty inside the the ego circle 
         * @param p1 Ending point of the first line; must be striclty outside the the ego circle 
         * @param p2 Ending point of the second line; must be striclty outside the the ego circle 
         */
        void set_keyhole(Vector2d xc, double r, Vector2d p0, Vector2d p1, Vector2d p2);

        /**
         * @brief Set the parameters of the keyhole with semi-parallel lines.
         * 
         * @details This method is used to reset the parameters of the keyhole with semi-parallel lines after the object is instantiated.
         * 
         * @param xc Center of the ego circle
         * @param r Radius of the ego circle 
         * @param q1 Starting point for the first line; must be on the circumference of the ego cricle
         * @param q2 Starting point for the second line; must be on the circumference of the ego cricle
         * @param p1 Ending point of the first line; must be striclty outside the the ego circle 
         * @param p2 Ending point of the second line; must be striclty outside the the ego circle 
         */
        void set_keyhole(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d p1, Vector2d p2);

        /**
         * @brief Get the circle points for plotting.
         * 
         * @return MatrixXd A two column matrix. Each row is a point.
         */
        MatrixXd get_circle_points();

        /**
         * @brief Get the line points for plotting
         * 
         * @param line1 True for the first line, and false for the second line.
         * @return MatrixXd A two column matrix. Each row is a point.
         */
        MatrixXd get_line_points(bool line1);

        /**
         * @brief Get the circle training points.
         *
         * @param safe_points a two coloumn matrix for safe points
         * @param unsafe_points a two coloumn matrix for unsafe points
         */
        void get_circle_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points);

        /**
         * @brief Get the line training points.
         * 
         * @param safe_points a two coloumn matrix for safe points
         * @param unsafe_points  a two coloumn matrix for unsafe points
         * @param line1 True for the first line, and false for the second line.
         */
        void get_line_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points, bool line1);

        void get_line_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points, bool line1, Vector2d p);
        void get_line_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points, bool line1, Vector2d q, Vector2d p);

        /**
         * @brief Synthesize BF using LP.
         * 
         * @param rbf_on true for using rbf in mode and false for using relu(circ) in model
         */
        bool synthesize(const bool &rbf_on);

        /**
         * @brief Evaluate BF at point x.
         * 
         * @param x input point x. 
         * @return double 
         */
        double evaluate_bf(Vector2d x);

        /**
         * @brief Calculates the gradient
         * 
         * @param x input xy-position
         * @return Vector2d Gradient of bf
         */
        Vector2d grad_bf(Vector2d x);

        /**
         * @brief Solve CBF-QP
         *
         * @param x input xy-position
         * @param f passive dynamics; evaluated externally
         * @param g control matrix; evaluated externally
         * @param ur control input
         * @param gamma tuning parameter for CBF; initialy set to 1.0 and then tune.
         * @return Vector2d calculated control
         */
        Vector2d cbf_qp(Vector3d state, Vector2d ur, double gamma, double k_w, double th_max);

        pair<vector<double>, vector<double>> get_contour(double levelset);

        void get_param(VectorXd& a, Vector2d& c1, Vector2d& c2, double& d1, double& d2);
        void get_param(VectorXd& a, Vector2d& c1, Vector2d& c2, Vector2d& c3, Vector2d& c4, Vector2d& c5, double& d1, double& d2, double& d3, double& d4, double& d5);
        VectorXd get_a();
        double get_r();
        Vector2d get_xc();

        friend std::ostream &operator<<(std::ostream &os, const Keyhole &keyhole);

        Vector2d get_intersection_(Vector2d xc, double r, Vector2d p0, Vector2d p1);

        void model(Vector2d x, bool rbf_on, VectorXd &g, MatrixXd &lims, VectorXd &cost);
        MatrixXd safe_points_syn_, unsafe_points_syn_;

        bool initialized()
        {
            return initialized_;
        }

        matplot::contour_line_type contour_line_keyhole(const matplot::vector_2d &X, const matplot::vector_2d &Y,
                                                        const matplot::vector_2d &Z, double level) {
            // create_contour is an external algorithm that seems to be generating
            // incoherent contour lines sometimes. So we regenerate the contour
            // until we get sometime that makes sense (something at least not out of
            // range). We then make sure this contour line matches the next contour
            // line.
            double x_min = matplot::min(X);
            double x_max = matplot::max(X);
            double y_min = matplot::min(Y);
            double y_max = matplot::max(Y);

            auto contour_is_in_bounds = [&](const matplot::contour_line_type &c) {
                double cx_min = matplot::min(c.first);
                double cx_max = matplot::max(c.first);
                double cy_min = matplot::min(c.second);
                double cy_max = matplot::max(c.second);
                const bool xminok = cx_min >= x_min;
                const bool xmaxok = cx_max <= x_max;
                const bool yminok = cy_min >= y_min;
                const bool ymaxok = cy_max <= y_max;

                // ROS_INFO_STREAM(xminok << " " << xmaxok << " " << yminok << " " << ymaxok);
                return (xminok && xmaxok && yminok && ymaxok);
            };

            auto contour_line_in_bounds = [&]() {
                matplot::QuadContourGenerator contour_generator(X, Y, Z, false, 0);
                auto c = contour_generator.create_contour(level);
                size_t attempts = 0;
                while (!contour_is_in_bounds(c) && attempts < 10) {
                    std::cerr << "Contour out of bounds" << std::endl;
                    matplot::QuadContourGenerator contour_generator2(X, Y, Z, false, 0);
                    c = contour_generator2.create_contour(level);
                    ++attempts;
                }
                return c;
            };

            auto c = contour_line_in_bounds();

            return c;
        }

        std::vector<matplot::contour_line_type> contourc_keyhole(const matplot::vector_2d &x,
                                                                 const matplot::vector_2d &y,
                                                                 const matplot::vector_2d &z,
                                                                 const matplot::vector_1d &levels) {
            std::vector<matplot::contour_line_type> lines;
            for (const double &level : levels) {
                lines.emplace_back(contour_line_keyhole(x, y, z, level));
            }
            return lines;
        }

    private:
        bool initialized_, same_keyhole_ = false;

        Vector2d xc_;
        Vector2d p0_;
        Vector2d p1_, p1_syn_;
        Vector2d p2_, p2_syn_;
        double r_;

        Vector2d q1_;
        Vector2d q2_;

        VectorXd a_;
        Vector2d c1_, c2_, c3_;
        Vector2d c4_{0.0,0.0};
        Vector2d c5_{0.0,0.0};
        double d1_, d2_, d3_;
        double d4_ = 0.0;
        double d5_ = 0.0;

        double tol_ = 1e-3;
        double inf_ = 9999.0;

        double ang_inc_ = 10.0*M_PI/180.0;
        double offset_ = 0.05;
        double offset_const_ = 0.03;
        double offset_lim_ = 0.05;

        bool synthesis_done_ = false;

        bool mode_triangle_on_;

        bool rbf_on_;

        bool extend_gap_ = false;

        MatrixXd c_safe_, c_unsafe_, l1_safe_, l1_unsafe_, l2_safe_, l2_unsafe_;

        void check_parameters_(Vector2d xc, double r, Vector2d p0, Vector2d p1, Vector2d p2);
        void check_parameters_parallel_(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d p1, Vector2d p2);

        // Vector2d get_intersection_(Vector2d xc, double r, Vector2d p0, Vector2d p1);

        MatrixXd sweep_circle_(Vector2d t1, Vector2d t2, Vector2d xc, double ang_inc, double r);

        double relu_line_(Vector2d x, Vector2d coe, double d);

        double rbf_(Vector2d x, Vector2d xc, double beta);

        double relu_circle_(Vector2d x, Vector2d xc, double r);

        void get_line_ceofficients_(Vector2d p0, Vector2d p1, bool left, Vector2d &c, double &d);

        bool is_in_(const double &x, const double &x1, const double &x2);

        MatrixXd get_new_safe_(MatrixXd l_safe, Vector2d c, double d);

        void extend_parameters_(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d &p1, Vector2d &p2);

        static autodiff::real bf_model_(autodiff::ArrayXreal &x, Keyhole *cbf);
        
        void correct_corner_points_(MatrixXd &c_safe, MatrixXd &l1_safe, MatrixXd &l2_safe);

        MatrixXd removeRow_(MatrixXd &mat, const int &rowNum);
};

}
}

#endif
