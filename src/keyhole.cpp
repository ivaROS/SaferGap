#include "potential_gap_mpc/keyhole.hpp"

using namespace operations_research;

namespace pg_mpc_local_planner{
namespace keyhole{

/********************************************************* 
**********************************************************/
Keyhole::Keyhole()
{
    initialized_ = false;
    same_keyhole_ = false;
}

Keyhole::~Keyhole()
{
    initialized_ = false;
    same_keyhole_ = false;
}


/********************************************************* 
**********************************************************/
Keyhole::Keyhole(Vector2d xc, double r, Vector2d p0, Vector2d p1, Vector2d p2){
    check_parameters_(xc, r, p0, p1, p2);
    xc_ = xc;
    r_ = r;
    p0_ = p0;
    p1_ = p1;
    p2_ = p2;
    offset_ = r*offset_const_;
    mode_triangle_on_ = true;
    initialized_ = true;
    same_keyhole_ = false;
}

Keyhole::Keyhole(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d p1, Vector2d p2){
    check_parameters_parallel_(xc, r, q1, q2, p1, p2);
    extend_parameters_(xc, r, q1, q2, p1, p2);
    xc_ = xc;
    r_ = r;
    q1_ = q1;
    q2_ = q2;
    p1_ = p1;
    p2_ = p2;
    offset_ = r*offset_const_;
    mode_triangle_on_ = false;
    initialized_ = true;
    same_keyhole_ = false;
}

/********************************************************* 
**********************************************************/
void Keyhole::set_keyhole(Vector2d xc, double r, Vector2d p0, Vector2d p1, Vector2d p2){
    if(xc == xc_ && r == r_ && p0 == p0_ && p1 == p1_ && p2 == p2_ && synthesis_done_)
    {
        same_keyhole_ = true;
        // synthesis_done_ = true;
    }
    else
    {
        check_parameters_(xc, r, p0, p1, p2);
        xc_ = xc;
        r_ = r;
        p0_ = p0;
        p1_ = p1;
        p2_ = p2;
        p1_syn_ = p1;
        p2_syn_ = p2;
        offset_ = r*offset_const_;
        same_keyhole_ = false;
        synthesis_done_ = false;
    }
    mode_triangle_on_ = true;
    initialized_ = true;
}

/********************************************************* 
**********************************************************/
void Keyhole::set_keyhole(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d p1, Vector2d p2){
    if(xc == xc_ && r == r_ && q1 == q1_ && q2 == q2_ && p1 == p1_ && p2 == p2_ && synthesis_done_)
    {
        same_keyhole_ = true;
        // synthesis_done_ = true;
    }
    else
    {
        check_parameters_parallel_(xc, r, q1, q2, p1, p2);
        extend_parameters_(xc, r, q1, q2, p1, p2);
        xc_ = xc;
        r_ = r;
        q1_ = q1;
        q2_ = q2;
        p1_ = p1;
        p2_ = p2;
        p1_syn_ = p1;
        p2_syn_ = p2;
        offset_ = r*offset_const_;
        same_keyhole_ = false;
        synthesis_done_ = false;
    }
    mode_triangle_on_ = false;
    initialized_ = true;
}

/********************************************************* 
**********************************************************/
void Keyhole::get_circle_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points){
    Vector2d t1;
    Vector2d t2;

    if(mode_triangle_on_){
        t1 = get_intersection_(xc_, r_, p0_, p1_);
        t2 = get_intersection_(xc_, r_, p0_, p2_);
    }else{
        t1 = q1_;
        t2 = q2_;
    }

    unsafe_points = sweep_circle_(t1, t2, xc_, ang_inc_, r_);
    MatrixXd grad = unsafe_points - xc_.transpose().replicate(unsafe_points.rows(),1);
    grad = (grad * 2.0).rowwise().normalized() * offset_;
    safe_points = unsafe_points - grad;
}

/********************************************************* 
**********************************************************/
void Keyhole::get_line_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points, bool line1){
    Vector2d p1;
    double s, m, d, my;
    VectorXd x_inc, y_inc;
    MatrixXd grad;

    if(line1){
        p1 = p1_;
    }else{
        p1 = p2_;
    }

    Vector2d t;
    if(mode_triangle_on_){
        t = get_intersection_(xc_, r_, p0_, p1);
    }else{
        if(line1){
            t = q1_;
            p0_ = q1_;
        }else{
            t = q2_;
            p0_ = q2_;
        }
    }

    Vector2d tmp = p1-p0_;

    if(line1){
        s = 1.0;
        if(tmp(0)<0){ // if in the left half
            s = -1.0;
        }
    }else{ // if in the right half
        s = 1.0;
        if(tmp(0)>0){
            s = -1.0;
        }
    }

    double inc_len = ang_inc_*r_;
    int inc_num = (int)ceil((t-p1).norm()/inc_len)+1;
    if(inc_num<3){
        inc_num = 3;
    }

    if(abs(p1(0)-p0_(0))>tol_){
        m = (p1(1)-p0_(1))/(p1(0)-p0_(0));
        d = p0_(1) - m*p0_(0);
        x_inc = VectorXd::LinSpaced(inc_num, t(0), p1(0));
        y_inc = x_inc*m + VectorXd::Ones(inc_num)*d;
        my = 1.0;
    }else{//vertical line
        m = 1;
        x_inc = VectorXd::Ones(inc_num)*(p0_(0)+p1(0))/2;
        y_inc = VectorXd::LinSpaced(inc_num, t(1), p1(1));
        my = 0.0;
        if(line1){
            if(tmp(1)>0){
                s = 1.0;
            }else{
                s = -1.0;
            }
        }else{
            if(tmp(1)>0){
                s = -1.0;
            }else{
                s = 1.0;
            }
        }
    }

    unsafe_points.resize(inc_num, 2);
    unsafe_points << x_inc, y_inc;
    grad.resize(inc_num, 2);
    grad << VectorXd::Ones(inc_num)*(-m), VectorXd::Ones(inc_num)*my;
    grad = grad.rowwise().normalized()*offset_;
    safe_points.resize(inc_num, 2);
    safe_points = unsafe_points + s*grad;
}

/********************************************************* 
**********************************************************/
void Keyhole::get_line_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points, bool line1, Vector2d p){
    Vector2d p1;
    double s, m, d, my;
    VectorXd x_inc, y_inc;
    MatrixXd grad;

    if(line1){
        p1 = p;
    }else{
        p1 = p;
    }

    Vector2d t;
    if(mode_triangle_on_){
        t = get_intersection_(xc_, r_, p0_, p1);
    }else{
        if(line1){
            t = q1_;
            p0_ = q1_;
        }else{
            t = q2_;
            p0_ = q2_;
        }
    }

    Vector2d tmp = p1-p0_;

    if(line1){
        s = 1.0;
        if(tmp(0)<0){ // if in the left half
            s = -1.0;
        }
    }else{ // if in the right half
        s = 1.0;
        if(tmp(0)>0){
            s = -1.0;
        }
    }

    double inc_len = ang_inc_*r_;
    int inc_num = (int)ceil((t-p1).norm()/inc_len)+1;
    if(inc_num<3){
        inc_num = 3;
    }

    if(abs(p1(0)-p0_(0))>tol_){
        m = (p1(1)-p0_(1))/(p1(0)-p0_(0));
        d = p0_(1) - m*p0_(0);
        x_inc = VectorXd::LinSpaced(inc_num, t(0), p1(0));
        y_inc = x_inc*m + VectorXd::Ones(inc_num)*d;
        my = 1.0;
    }else{//vertical line
        m = 1;
        x_inc = VectorXd::Ones(inc_num)*(p0_(0)+p1(0))/2;
        y_inc = VectorXd::LinSpaced(inc_num, t(1), p1(1));
        my = 0.0;
        if(line1){
            if(tmp(1)>0){
                s = 1.0;
            }else{
                s = -1.0;
            }
        }else{
            if(tmp(1)>0){
                s = -1.0;
            }else{
                s = 1.0;
            }
        }
    }

    unsafe_points.resize(inc_num, 2);
    unsafe_points << x_inc, y_inc;
    grad.resize(inc_num, 2);
    grad << VectorXd::Ones(inc_num)*(-m), VectorXd::Ones(inc_num)*my;
    grad = grad.rowwise().normalized()*offset_;
    safe_points.resize(inc_num, 2);
    safe_points = unsafe_points + s*grad;
}

/********************************************************* 
**********************************************************/
void Keyhole::get_line_train_points(MatrixXd &safe_points, MatrixXd &unsafe_points, bool line1, Vector2d q, Vector2d p){
    Vector2d p1;
    double s, m, d, my;
    VectorXd x_inc, y_inc;
    MatrixXd grad;

    if(line1){
        p1 = p;
    }else{
        p1 = p;
    }

    Vector2d t=q;
    // if(mode_triangle_on_){
    //     t = get_intersection_(xc_, r_, p0_, p1);
    // }else{
    //     if(line1){
    //         t = q1_;
    //         p0_ = q1_;
    //     }else{
    //         t = q2_;
    //         p0_ = q2_;
    //     }
    // }

    Vector2d tmp = p1-q;

    if(line1){
        s = 1.0;
        if(tmp(0)<0){ // if in the left half
            s = -1.0;
        }
    }else{ // if in the right half
        s = 1.0;
        if(tmp(0)>0){
            s = -1.0;
        }
    }

    double inc_len = ang_inc_*r_;
    int inc_num = (int)ceil((t-p1).norm()/inc_len)+1;
    if(inc_num<3){
        inc_num = 3;
    }

    if(abs(p1(0)-q(0))>tol_){
        m = (p1(1)-q(1))/(p1(0)-q(0));
        d = q(1) - m*q(0);
        x_inc = VectorXd::LinSpaced(inc_num, t(0), p1(0));
        y_inc = x_inc*m + VectorXd::Ones(inc_num)*d;
        my = 1.0;
    }else{//vertical line
        m = 1;
        x_inc = VectorXd::Ones(inc_num)*(q(0)+p1(0))/2;
        y_inc = VectorXd::LinSpaced(inc_num, t(1), p1(1));
        my = 0.0;
        if(line1){
            if(tmp(1)>0){
                s = 1.0;
            }else{
                s = -1.0;
            }
        }else{
            if(tmp(1)>0){
                s = -1.0;
            }else{
                s = 1.0;
            }
        }
    }

    unsafe_points.resize(inc_num, 2);
    unsafe_points << x_inc, y_inc;
    grad.resize(inc_num, 2);
    grad << VectorXd::Ones(inc_num)*(-m), VectorXd::Ones(inc_num)*my;
    grad = grad.rowwise().normalized()*offset_;
    safe_points.resize(inc_num, 2);
    safe_points = unsafe_points + s*grad;
}

/********************************************************* 
**********************************************************/
MatrixXd Keyhole::get_circle_points(){
    int const inc = 100;
    // VectorXd x1 = VectorXd::Linspaced(inc,1,-1);
    // VectorXd x2 = VectorXd::Linspaced(inc,-1,1);
    VectorXd x1; x1.setLinSpaced(inc, 1, -1);
    VectorXd x2; x2.setLinSpaced(inc, -1, 1);
    VectorXd y1 = sqrt(pow(r_,1)-x1.array().pow(2));
    VectorXd y2 = -sqrt(pow(r_,1)-x2.array().pow(2));

    VectorXd x(inc*2); x << x1, x2;
    VectorXd y(inc*2); y << y1, y2;

    MatrixXd xy(inc*2,2);
    xy << x, y;
    
    return xy.array().rowwise() + xc_.transpose().array() ;
}

/********************************************************* 
**********************************************************/
MatrixXd Keyhole::get_line_points(bool line1){
    Vector2d pi, po, pv;
    VectorXd x, y;
    double m;
    double d;
    int inc = 100;

    // get the point in/out the circle (initial point)
    if(mode_triangle_on_){
        pi = p0_;
    }
    else{
        if (line1){
            pi = q1_;
        }
        else{
            pi = q2_;
        }
    }

    // get second point outside the circle
    if (line1){
        po = p1_;
    }
    else{
        po = p2_;
    }

    // check for vertical line
    pv = po - pi;
    if(abs(pv(0))<tol_){// vertical line
        double tmp = (pi(0)+po(0))/2;
        x = Matrix<double, 1, 1>(tmp).replicate(inc,1);
        y.setLinSpaced(inc, pi(1), po(1));
    }else{
        m = (po(1) - pi(1)) / (po(0) - pi(0));
        d = pi(1) - m*pi(0);
        x.setLinSpaced(inc, pi(0), po(0));
        y = m*x + VectorXd::Ones(inc)*d;
    }

    MatrixXd xy(inc,2);
    xy << x, y;

    return xy;
}

/********************************************************* 
**********************************************************/
bool Keyhole::synthesize(const bool &rbf_on){
    if(!initialized_)
    {
        ROS_WARN_STREAM("The keyhole is not initialized.");
        return false;
    }

    if(same_keyhole_)
    {
        ROS_INFO_STREAM("The keyhole is the same, no need to synthesize.");
        return synthesis_done_;
    }

    rbf_on_ = rbf_on;
    extend_gap_ = false;

    MatrixXd c_safe, c_unsafe, l1_safe, l1_unsafe, l2_safe, l2_unsafe, l3_safe, l3_unsafe;
    get_circle_train_points(c_safe, c_unsafe);
    get_line_train_points(l1_safe, l1_unsafe, true);
    get_line_train_points(l2_safe, l2_unsafe, false);
    correct_corner_points_(c_safe, l1_safe, l2_safe);
    c_safe_ = c_safe;
    c_unsafe_ = c_unsafe;
    l1_safe_ = l1_safe;
    l1_unsafe_ = l1_unsafe;
    l2_safe_ = l2_safe;
    l2_unsafe_ = l2_unsafe;

    int num_safe = c_safe.rows() + l1_safe.rows() + l2_safe.rows();
    int num_unsafe = c_unsafe.rows() + l1_unsafe.rows() + l2_unsafe.rows();
    int num_safe_syn, num_unsafe_syn;

    MatrixXd safe_points(num_safe, 2);
    safe_points << c_safe, l1_safe, l2_safe;
    MatrixXd unsafe_points(num_unsafe, 2);
    unsafe_points << c_unsafe, l1_unsafe, l2_unsafe;
    MatrixXd safe_points_syn, unsafe_points_syn;

    Vector2d c1, c2, c3, pv, pi, qv;
    Vector2d p1s, p2s, c4, c5;
    double d1, d2, d3, d4, d5;

    // first line
    if(mode_triangle_on_){
        pi = p0_;
    }else{
        pi = q1_;
    }
    get_line_ceofficients_(pi, p1_, true, c1, d1);

    // second line
    if(mode_triangle_on_){
        pi = p0_;
    }else{
        pi = q2_;
    }
    get_line_ceofficients_(pi, p2_, false, c2, d2);

    c1_ = c1;
    c2_ = c2;
    d1_ = d1;
    d2_ = d2;

    //thrid line
    double x_int, y_int, tmp_int;
    if(!mode_triangle_on_){
        get_line_ceofficients_(q1_, q2_, false, c3, d3);
        c3_ = c3;
        d3_ = d3;

        tmp_int = c1(1)*c2(0) - c2(1)*c1(0);
        if(abs(tmp_int)<=1e-4){

        }else{
            x_int = (c2(1)*d1-c1(1)*d2)/tmp_int;
            y_int = -(c1(0)*x_int+d1)/c1(1);

            if((is_in_(x_int, q1_(0), p1_(0)))&&(is_in_(x_int, q2_(0), p2_(0)))){ // lines intersecting
                p1_syn_ = Vector2d{x_int, y_int};
                p2_syn_ = Vector2d{x_int, y_int};
                get_line_train_points(l1_safe, l1_unsafe, true, p1_syn_);
                get_line_train_points(l2_safe, l2_unsafe, false, p2_syn_);
                // l1_safe = get_new_safe_(l1_safe, c2, d2);
                // l2_safe = get_new_safe_(l2_safe, c1, d1);
                // l1_safe = removeRow_(l1_safe, 0);
                // l2_safe = removeRow_(l2_safe, 0);
                // l1_safe = removeRow_(l1_safe, l1_safe.rows()-1);
                // l2_safe = removeRow_(l2_safe, l2_safe.rows()-1);
                MatrixXd tmp = l1_safe.block(1,0, l1_safe.rows()-2, 2);
                l1_safe.resize(tmp.rows(), 2);
                l1_safe = tmp;
                tmp.resize(l2_safe.rows() - 2, 2);
                tmp = l2_safe.block(1, 0, l2_safe.rows() - 2, 2);
                l2_safe.resize(tmp.rows(), 2);
                l2_safe = tmp;
                safe_points_syn.resize(c_safe.rows() + l1_safe.rows() + l2_safe.rows(), 2);
                safe_points_syn << c_safe, l1_safe, l2_safe;
                unsafe_points_syn.resize(c_unsafe.rows()+l1_unsafe.rows()+l2_unsafe.rows(),2);
                unsafe_points_syn << c_unsafe, l1_unsafe, l2_unsafe;

                c4_ << 0.0, 0.0;
                c5_ << 0.0, 0.0;
                d4_ = 0.0;
                d5_ = 0.0;
            }
            else if ((is_in_(x_int, q1_(0), p1_(0))) && (!is_in_(x_int, q2_(0), p2_(0))))
            { // line 2 extension interset lin 1
                // cout << "extending line2 to intersect line 1" << endl;
                // p1_syn_ = q1_ + (p1_-q1_)*0.9;
                // get_line_train_points(l1_safe, l1_unsafe, true, p1_syn_);
                // l1_safe = removeRow_(l1_safe, 0);

                double m1 = (p1_(1)-q1_(1))/(p1_(0)-q1_(0));
                double d1 = q1_(1)-m1*q1_(0);

                double tmp = (p1_(1)-p2_(1))/m1 + p2_(0);
                p2s << tmp, p1_(1);
                get_line_ceofficients_(p2_, p2s, false, c4, d4);

                double m2 = (q1_(1)-q2_(1))/(q1_(0)-q2_(0));
                double d2 = p2_(1)-m2*p2_(0);
                
                tmp = (d2-d1)/(m1-m2);
                p1s << tmp, tmp*m1+d1;
                get_line_ceofficients_(p1s, p2_, false, c5, d5);

                c4_ = c4;
                c5_ = c5;
                d4_ = d4;
                d5_ = d5;
                extend_gap_ = true;
                get_line_train_points(l3_safe, l3_unsafe, false, p2_, p2s);
                safe_points_syn.resize(c_safe.rows()+l1_safe.rows()+l2_safe.rows()+l3_safe.rows(),2);
                unsafe_points_syn.resize(c_unsafe.rows()+l1_unsafe.rows()+l2_unsafe.rows()+l3_unsafe.rows(),2);
                safe_points_syn << c_safe, l1_safe, l2_safe, l3_safe;
                unsafe_points_syn << c_unsafe, l1_unsafe, l2_unsafe, l3_unsafe;
            }
            else if ((!is_in_(x_int, q1_(0), p1_(0))) && (is_in_(x_int, q2_(0), p2_(0))))
            { // line 1 extension interset lin 2
                // cout << "extending line1 to intersect line 2" << endl;
                // p2_syn_ = q2_ + (p2_-q2_)*0.9;
                // get_line_train_points(l2_safe, l2_unsafe, false, p2_syn_);
                // l2_safe = removeRow_(l2_safe, 0);

                double m2 = (p2_(1)-q2_(1))/(p2_(0)-q2_(0));
                double d2 = q2_(1)-m2*q2_(0);

                double tmp = (p2_(1)-p1_(1))/m2 + p1_(0);
                p1s << tmp, p2_(1);
                get_line_ceofficients_(p1_, p1s, true, c4, d4);

                double m1 = (q1_(1)-q2_(1))/(q1_(0)-q2_(0));
                double d1 = q1_(1)-m1*q1_(0);
                
                tmp = (d2-d1)/(m1-m2);
                p2s << tmp, tmp*m1+d1;
                get_line_ceofficients_(p1_, p2s, false, c5, d5);

                c4_ = c4;
                c5_ = c5;
                d4_ = d4;
                d5_ = d5;
                extend_gap_ = true;
                get_line_train_points(l3_safe, l3_unsafe, true, p1_, p1s);
                safe_points_syn.resize(c_safe.rows()+l1_safe.rows()+l2_safe.rows()+l3_safe.rows(),2);
                unsafe_points_syn.resize(c_unsafe.rows()+l1_unsafe.rows()+l2_unsafe.rows()+l3_unsafe.rows(),2);
                safe_points_syn << c_safe, l1_safe, l2_safe, l3_safe;
                unsafe_points_syn << c_unsafe, l1_unsafe, l2_unsafe, l3_unsafe;
            }
            else
            { // no intersection
                // check if intersection point is too close
                if((p1_-Vector2d{x_int,y_int}).norm()<2*offset_){
                    p1_syn_ = q1_ + (p1_-q1_) - (p1_-q1_)/(p1_-q1_).norm()*offset_*2;
                    get_line_train_points(l1_safe, l1_unsafe, true, p1_syn_);
                    l1_safe = removeRow_(l1_safe, 0);
                    // cout << "shortened line 1" << endl;
                }
                if((p2_-Vector2d{x_int,y_int}).norm()<2*offset_){
                    p2_syn_ = q2_ + (p2_-q2_) - (p2_-q2_)/(p2_-q2_).norm()*offset_*2;
                    get_line_train_points(l2_safe, l2_unsafe, false, p2_syn_);
                    l2_safe = removeRow_(l2_safe, 0);
                    // cout << "shortened line 2" << endl;
                }
                // safe_points_syn = safe_points;
                // unsafe_points_syn = unsafe_points;
                safe_points_syn.resize(c_safe.rows()+l1_safe.rows()+l2_safe.rows(),2);
                safe_points_syn << c_safe, l1_safe, l2_safe;
                unsafe_points_syn.resize(c_unsafe.rows()+l1_unsafe.rows()+l2_unsafe.rows(),2);
                unsafe_points_syn << c_unsafe, l1_unsafe, l2_unsafe;
                c4_ << 0.0, 0.0;
                c5_ << 0.0, 0.0;
                d4_ = 0.0;
                d5_ = 0.0;
            }
        }
    }else{
        safe_points_syn = safe_points;
        unsafe_points_syn = unsafe_points;
    }
    safe_points_syn_ = safe_points_syn;
    unsafe_points_syn_ = unsafe_points_syn;
    num_safe_syn = safe_points_syn.rows();
    num_unsafe_syn = unsafe_points_syn.rows();

    // Import the linear solver wrapper,
    // declare the LP solver,
    unique_ptr<MPSolver> solver(MPSolver::CreateSolver("GLOP"));

    // define the variables,
    const double inf = solver->infinity();
    int a_num, a_model;
    bool soft_on = false;
    VectorXd g, cost;
    MatrixXd lims;
    model(xc_, rbf_on_, g, lims, cost);

    a_model = g.size();
    if(soft_on){
        a_num = a_model + num_safe_syn;
    }else{
        a_num = a_model;
    }

    MPVariable *a[a_num];
    for(int i=0; i<a_model; i++){
        double low, high;
        low = lims(i,0);
        high = lims(i,1);

        if(abs(low-inf_)<tol_){
            low = inf;
        }else if(abs(low-(-inf_))<tol_){
            low = -inf;
        }

        if(abs(high-inf_)<tol_){
            high = inf;
        }else if(abs(high-(-inf_))<tol_){
            high = -inf;
        }

        a[i] = solver->MakeNumVar(low, high, "a"+std::to_string(i+1));
    }
    if(soft_on){
        for(int i=a_model; i<a_num; i++){
            a[i] = solver->MakeNumVar(0.0, inf, "xi"+std::to_string(i+1-a_model));
        }
    }

    // define the objective,
    MPObjective* const obj = solver->MutableObjective();

    // define constraints
    MPConstraint *c[num_safe_syn + num_unsafe_syn];
    Vector2d tmpx;
    int k = a_model;

    // safe points
    for (int i = 0; i < num_safe_syn; i++){
        tmpx << safe_points_syn(i, 0), safe_points_syn(i, 1);
        model(tmpx, rbf_on_, g, lims, cost);
        c[i] = solver->MakeRowConstraint(1.0, inf);
        for(int j=0; j<a_model; j++){
            c[i]->SetCoefficient(a[j], g(j));
        }
        if(soft_on){
            for(int j=a_model; j<a_num; j++){
                if(j==k){
                    c[i]->SetCoefficient(a[j], 1);
                }else{
                    c[i]->SetCoefficient(a[j], 0);
                }
            }
            k++;
        }
    }

    // unsafe points
    for (int i = 0; i < num_unsafe_syn; i++){
        tmpx << unsafe_points_syn(i,0), unsafe_points_syn(i,1);
        model(tmpx, rbf_on_, g, lims, cost);
        c[i] = solver->MakeRowConstraint(-inf, -1.0);
        for(int j=0; j<a_model; j++){
            c[i]->SetCoefficient(a[j], g(j));
        }
        if(soft_on){
            for(int j=a_model; j<a_num; j++){
                c[i]->SetCoefficient(a[j], 0);
            }
        }
    }

    // set cost function
    for(int i=0; i<a_model; i++){
        obj->SetCoefficient(a[i], cost(i));
    }
    if(soft_on){
        for(int j=a_model; j<a_num; j++){
            obj->SetCoefficient(a[j], 1);
        }
    }
    obj->SetMinimization();

    bool set_param = solver->SetSolverSpecificParametersAsString("solution_feasibility_tolerance: 2e-5");

// solver->EnableOutput();
  // call the LP solver;
  const MPSolver::ResultStatus result_status = solver->Solve();
  if (result_status != MPSolver::OPTIMAL) {
    ROS_WARN_STREAM("The problem does not have an optimal solution!");
    a_.resize(a_model);
    for(int i=0; i<a_model-1; i++){
        a_(i) = 0;
    }
    a_(a_model-1) = 1.;
    synthesis_done_ = false;
    return false;
  }

  a_.resize(a_model);
  for(int i=0; i<a_model; i++){
    a_(i) = a[i]->solution_value();
  }

  if(soft_on){
    cout << "xi=[";
    for(int i=a_model; i<a_num; i++){
        cout << a[i]->solution_value() << ", ";
    }
    cout << "]\n";
  }

  synthesis_done_ = true;
  return true;
}

/********************************************************* 
**********************************************************/
double Keyhole::evaluate_bf(Vector2d x){
    if(!synthesis_done_){
        ROS_WARN_STREAM("[evaluate_bf] BF is not synthesized yet!");
        return 1;
    }

    VectorXd g, cost;
    MatrixXd lims;
    model(x, rbf_on_, g, lims, cost);

    return a_.transpose()*g;
}

/********************************************************* 
**********************************************************/
void Keyhole::model(Vector2d x, bool rbf_on, VectorXd &g, MatrixXd &lims, VectorXd &cost){
    // VectorXd g, cost;
    // MatrixXd lims;
    double relu1, relu2, relu3, relu4, relu5, cir;
    relu1 = relu_line_(x,c1_,d1_);
    relu2 = relu_line_(x,c2_,d2_);
    relu3 = relu_line_(x,c3_,d3_);
    relu4 = relu_line_(x,c4_,d4_);
    relu5 = relu_line_(x,c5_,d5_);

    if(rbf_on){
        cir = rbf_(x, xc_, 1.0/r_);
    }else{
        cir = relu_circle_(x, xc_, r_);
    }

    if(mode_triangle_on_){
        g.resize(5);
        g << relu1,
             relu2,
             relu1*relu2,
             cir,
             1;
        lims.resize(5,2);
        lims << 0.0,     inf_,
                0.0,     inf_,
                0.0,     inf_,
                0.0,     inf_,
                -inf_, 0.0;
        cost.resize(5);
        cost << 10, 10, 1, 1, 0;
    }else{
        int num = 16;
            g.resize(num);
            g << relu1,
                 relu2,
                 relu3,
                 relu1*relu2,
                 relu1*relu2*relu3,
                 relu1*relu4*relu5,
                 relu2*relu4*relu5,
                 cir*relu1*relu4,
                 cir*relu2*relu4,
                 cir*relu1,
                 cir*relu2,
                 cir*relu3,
                 cir*relu1*relu2,
                 cir*relu1*relu2*relu3,
                 cir,
                 1;
            cost.resize(num);
            cost << 10000, 10000, 10000, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0;
        if(!extend_gap_){
            lims.resize(num,2);
            lims << 0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     0.0,
                    0.0,     0.0,
                    0.0,     0.0,
                    0.0,     0.0,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    -inf_, 0.0;
        }else{
            lims.resize(16,2);
            lims << 0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    0.0,     inf_,
                    -inf_, 0.0;
        }
    }

}

autodiff::real Keyhole::bf_model_(autodiff::ArrayXreal &x, Keyhole *cbf){
    // return 5*x(0) + 9*x(1);
    // cout << cbf->xc_ << endl;
    // return cbf->xc_(0)*x(0) + cbf->xc_(1)*x(1);
    autodiff::real relu1 = max(0.0, cbf->c1_(0)*x(0) + cbf->c1_(1)*x(1) + cbf->d1_);
    autodiff::real relu2 = max(0.0, cbf->c2_(0)*x(0) + cbf->c2_(1)*x(1) + cbf->d2_);
    autodiff::real relu3 = max(0.0, cbf->c3_(0)*x(0) + cbf->c3_(1)*x(1) + cbf->d3_);
    autodiff::real relu4 = max(0.0, cbf->c4_(0)*x(0) + cbf->c4_(1)*x(1) + cbf->d4_);
    autodiff::real relu5 = max(0.0, cbf->c5_(0)*x(0) + cbf->c5_(1)*x(1) + cbf->d5_);
    autodiff::real cir;
    if(cbf->rbf_on_){
        cir = exp(-(1/cbf->r_)*(pow(x(0)-cbf->xc_(0),2)+pow(x(1)-cbf->xc_(1),2)));
    }else{
        cir = max(0.0, -(pow(x(0)-cbf->xc_(0),2) + pow(x(1)-cbf->xc_(1),2) - pow(cbf->r_,2)));
    }    

    if(cbf->mode_triangle_on_){
        return cbf->a_(0)*relu1 + 
               cbf->a_(1)*relu2 +
               cbf->a_(2)*relu1*relu2 + 
               cbf->a_(3)*cir + 
               cbf->a_(4)*1.0;
    }else{
        return cbf->a_(0)*relu1 +
               cbf->a_(1)*relu2 +
               cbf->a_(2)*relu3 +
               cbf->a_(3)*relu1*relu2 +
               cbf->a_(4)*relu1*relu2*relu3 +
               cbf->a_(5)*relu1*relu4*relu5 +
               cbf->a_(6)*relu2*relu4*relu5 +
               cbf->a_(7)*cir*relu1*relu4 +
               cbf->a_(8)*cir*relu2*relu4 +
               cbf->a_(9)*cir*relu1 +
               cbf->a_(10)*cir*relu2 +
               cbf->a_(11)*cir*relu3 +
               cbf->a_(12)*cir*relu1*relu2 +
               cbf->a_(13)*cir*relu1*relu2*relu3 +
               cbf->a_(14)*cir +
               cbf->a_(15)*1.0;
    }
}

Vector2d Keyhole::grad_bf(Vector2d x){
    if(!synthesis_done_){
        ROS_WARN_STREAM("[grad_bf] BF is not synthesized yet!");
        return Eigen::Vector2d(0, 0);
    }

    autodiff::ArrayXreal tmpx(2);
    tmpx << x(0), x(1);
    // Vector2d g = gradient(bf_model_, wrt(tmpx), at(tmpx, a_, c1_, c2_, c3_, c4_, c5_, d1_, d2_, d3_, d4_, d5_, xc_, r_));
    Vector2d g = gradient(bf_model_, wrt(tmpx), at(tmpx, this));
    // cout << "g = [" << g.transpose() << "]" << endl;
    return g;

    // double l1 = c1_.transpose()*x + d1_;
    // double l2 = c2_.transpose()*x + d2_;
    // double l3 = c3_.transpose()*x + d3_;
    // double cir = -( (x-xc_).transpose()*(x-xc_)-pow(r_,2) );
    // Vector2d dl1, dl2, dl3, dl12, dl123, dcir;

    // if(mode_triangle_on_){
    //     if(l1<0.0){
    //         dl1 = Vector2d::Ones()*0.0;
    //     }else{
    //         dl1 = a_(0)*c1_;
    //     }

    //     if(l2<0.0){
    //         dl2 = Vector2d::Ones()*0.0;
    //     }else{
    //         dl2 = a_(1)*c2_;
    //     }

    //     if((l1<0.0)||(l2<0)){
    //         dl12 = Vector2d::Ones()*0.0;
    //     }else{
    //         dl12 = a_(2)*(c1_*c2_.transpose()*x + c2_*c1_.transpose()*x + d2_*c1_ + d1_*c2_);
    //     }
        
    //     if(rbf_on_){
    //         dcir = -a_(3)*2.0*1.0/r_*exp(-1.0/r_*pow((x-xc_).norm(),2))*(x-xc_);
    //     }else{
    //         if(cir<0.0){
    //             dcir = Vector2d::Ones()*0.0;
    //         }else{
    //             dcir = -a_(3)*2.0*(x-xc_);
    //         }
    //     }
    //     return dl1 + dl2 + dl12 + dcir;


    // }else{

    //     if(l1<0.0){
    //         dl1 = Vector2d::Ones()*0.0;
    //     }else{
    //         dl1 = a_(0)*c1_;
    //     }

    //     if(l2<0.0){
    //         dl2 = Vector2d::Ones()*0.0;
    //     }else{
    //         dl2 = a_(1)*c2_;
    //     }

    //     if(l3<0.0){
    //         dl3 = Vector2d::Ones()*0.0;
    //     }else{
    //         dl3 = a_(2)*c3_;
    //     }

    //     if((l1<0.0)||(l2<0)){
    //         dl12 = Vector2d::Ones()*0.0;
    //     }else{
    //         dl12 = a_(3)*(c1_*c2_.transpose()*x + c2_*c1_.transpose()*x + d2_*c1_ + d1_*c2_);
    //     }

    //     if((l1<0.0)||(l2<0.0)||(l3<0.0)){
    //         dl123 = Vector2d::Ones()*0.0;
    //     }else{
    //         Matrix2d c12, c13, c23,
    //                  c21, c31, c32;
    //         c12 = c1_*c2_.transpose();
    //         c21 = c12.transpose();
    //         c13 = c1_*c3_.transpose();
    //         c31 = c13.transpose();
    //         c23 = c2_*c3_.transpose();
    //         c32 = c23.transpose();

    //         dl123 = a_(4)*( d2_*(c31+c13)*x + d1_*(c32+c23)*x + d3_*(c12+c21)*x +
    //                         d1_*d2_*c3_ + d2_*d3_*c1_ + d1_*d3_*c2_ + 
    //                         (x.transpose()*c3_)*c12*x + (x.transpose()*c3_)*c21*x + c3_*(x.transpose()*c12*x) 
    //                         );
    //     }

    //     if(rbf_on_){
    //         dcir = -a_(5)*2.0*1.0/r_*exp(-1.0/r_*pow((x-xc_).norm(),2))*(x-xc_);
    //     }else{
    //         if(cir<0.0){
    //             dcir = Vector2d::Ones()*0.0;
    //         }else{
    //             dcir = -a_(5)*2.0*(x-xc_);
    //         }
    //     }

    //     return dl1 + dl2 + dl3 + dl12 + dl123 + dcir;
    // }
}

/********************************************************* 
**********************************************************/
Vector2d Keyhole::cbf_qp(Vector3d state, Vector2d ur, double gamma, double k_w, double th_max){
    if(!synthesis_done_)
    {
        ROS_WARN_STREAM("No optimal found, return the same control.");
        return ur;
    }

    if(abs(ur(0)) <= 1e-6 && abs(ur(1)) <= 1e-6)
    {
        return ur;
    }

    Vector2d x{state(0), state(1)};
    double th = state(2);
    MatrixXd g = MatrixXd::Identity(2,2);
    Vector2d f{0,0};

    double h = evaluate_bf(x);
    Vector2d dh = grad_bf(x);
    Vector2d u{cos(th)*ur(0), sin(th)*ur(0)};
    Vector2d k = g.transpose()*dh;
    Matrix3d A;
    Vector3d b;
    Vector3d tmp;
    double val = dh.dot(f) + dh.transpose()*g*u;
    if ((val) < (-gamma*h)){
        A << 1,     0,     k(0),
             0,     1,     k(1),
            -k(0), -k(1),  0;
        b << ur(0), ur(1), gamma*h+dh.dot(f);
        tmp = A.completeOrthogonalDecomposition().pseudoInverse()*b;
        u(0) = tmp(0);
        u(1) = tmp(1);
    }

    double th_new = atan2(u(1),u(0));
    if(th_new<0){
        th_new = th_new + KPI*2;
    }
    if(th<0){
        th = th + KPI*2;
    }
    
    double delta = th_new - th;
    if(abs(delta)>KPI){
        if(delta<0){
            delta = delta + KPI*2.0;
        }else{
            delta = delta - KPI*2.0;
        }
    }

    // double k_w = 1;
    // double k_v = 1;
    // double w_new = delta*k_w;
    double w_new = delta*k_w + ur(1);
    double v_new = Vector2d{u(0), u(1)}.norm();
    v_new = max(0.0, 1-abs(delta)/th_max)*v_new;

    u << v_new, w_new;

    return u;
}

pair<vector<double>, vector<double>> Keyhole::get_contour(double levelset){
    if(!synthesis_done_){
        ROS_WARN_STREAM("[get_contour] BF is not synthesized yet!");
        std::vector<double> zero_vec{0};
        return pair<vector<double>, vector<double>>(zero_vec, zero_vec);
    }

    int num_r = c_safe_.rows() + c_unsafe_.rows() + l1_safe_.rows() + l1_unsafe_.rows() + l2_safe_.rows() + l2_unsafe_.rows();
    MatrixXd tmp(num_r, 2);
    tmp << c_safe_, c_unsafe_, l1_safe_, l1_unsafe_, l2_safe_, l2_unsafe_;

    matplot::vector_1d x = matplot::linspace(tmp.col(0).minCoeff()-1, tmp.col(0).maxCoeff()+1, 100);
    matplot::vector_1d y = matplot::linspace(tmp.col(1).minCoeff()-1, tmp.col(1).maxCoeff()+1, 100);

    auto [X, Y] = matplot::meshgrid(x, y);
    matplot::vector_2d Z;
    Z = matplot::transform(X, Y, [&](double x, double y)
                    { return evaluate_bf(Vector2d{x,y}); });

    vector<matplot::contour_line_type> res = contourc_keyhole(X, Y, Z, std::vector<double>{levelset});

    return res[0];

}


/********************************************************* 
**********************************************************/
void Keyhole::get_param(VectorXd& a, Vector2d& c1, Vector2d& c2, double& d1, double& d2){
    if(!mode_triangle_on_){
        throw "You are using the wron method. The edge mode is enabled; thus use the overloaded method with c3 and d3 arguments";
    }
    a = a_;
    c1 = c1_;
    c2 = c2_;
    d1 = d1_;
    d2 = d2_;
}

void Keyhole::get_param(VectorXd& a, Vector2d& c1, Vector2d& c2, Vector2d& c3, Vector2d& c4, Vector2d& c5, double& d1, double& d2, double& d3, double& d4, double& d5){
    if(mode_triangle_on_){
        throw "You are using the wron method. The edge mode is disabled; thus use the overloaded method without c3 and d3 arguments";
    }
    a = a_;
    c1 = c1_;
    c2 = c2_;
    c3 = c3_;
    c4 = c4_;
    c5 = c5_;
    d1 = d1_;
    d2 = d2_;
    d3 = d3_;
    d4 = d4_;
    d5 = d5_;
}

VectorXd Keyhole::get_a(){
    return a_;
}

double Keyhole::get_r(){
    return r_;
}

Vector2d Keyhole::get_xc(){
    return xc_;
}

/********************************************************* 
**********************************************************/
void Keyhole::check_parameters_(Vector2d xc, double r, Vector2d p0, Vector2d p1, Vector2d p2){
    double tmp = 0.0;

    if(!(r>0.0)){
        throw std::invalid_argument("The radius r must be a positive number (r>0). r="+std::to_string(r)+" was the input value!");
    }
    tmp = (xc-p0).norm();
    if (!(tmp < r))
    {
        throw std::invalid_argument("The point p0 must be within the ego circle. The distance from xc to p0 is " + std::to_string(tmp) + ", which is greater or equal than r=" + std::to_string(r) + "!");
    }

    tmp = (xc-p1).norm();
    if(!(tmp>r)){
        throw std::invalid_argument("The point p1 must be outside the ego circle. The distance from xc to p1 is " + std::to_string(tmp) + ", which is less or equal than r=" + std::to_string(r) + "!");
    }

    tmp = (xc-p2).norm();
    if(!(tmp>r)){
        throw std::invalid_argument("The point p2 must be outside the ego circle. The distance from xc to p1 is " + std::to_string(tmp) + ", which is less or equal than r=" + std::to_string(r) + "!");
    }

}

void Keyhole::check_parameters_parallel_(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d p1, Vector2d p2){
    double tmp = 0.0;

    if(!(r>0.0)){
        throw std::invalid_argument("The radius r must be a positive number (r>0). r="+std::to_string(r)+" was the input value!");
    }

    tmp = (xc-q1).norm();
    if (!(abs(tmp-r)<1e-2))
    {
        throw std::invalid_argument("The point q1 must be on the circumference of the ego circle. The distance from xc to q1 is " + std::to_string(tmp) + ", which is greater or equal than r=" + std::to_string(r) + "!");
    }

    tmp = (xc-q2).norm();
    if (!(abs(tmp-r)<1e-2))
    {
        throw std::invalid_argument("The point q2 must be on the circumference of the ego circle. The distance from xc to q1 is " + std::to_string(tmp) + ", which is greater or equal than r=" + std::to_string(r) + "!");
    }

    tmp = (xc-p1).norm();
    if(!(tmp>r)){
        throw std::invalid_argument("The point p1 must be outside the ego circle. The distance from xc to p1 is " + std::to_string(tmp) + ", which is less or equal than r=" + std::to_string(r) + "!");
    }

    tmp = (xc-p2).norm();
    if(!(tmp>r)){
        throw std::invalid_argument("The point p2 must be outside the ego circle. The distance from xc to p1 is " + std::to_string(tmp) + ", which is less or equal than r=" + std::to_string(r) + "!");
    }  
}

/********************************************************* 
**********************************************************/
Vector2d Keyhole::get_intersection_(Vector2d xc, double r, Vector2d p0, Vector2d p1){
    p0 = p0 - xc;
    p1 = p1 - xc;
    double tmp = p0(0) - p1(0);

    double xs = 0;
    double ys = 0;
    Vector2d ps(0,0);

    double m = 0;
    double d = 0;

    if(abs(tmp)<1e-4){ // vertical line
        xs = (p1(0)+p0(0))/2;
        if((p1(1)-p0(1))>0){ //pointing up
            ys = sqrt(pow(r,2)-pow(xs,2));
        }else{
            ys = -sqrt(pow(r,2)-pow(xs,2));
        }
    }else if((p1(0)-p0(0))>0){ // right line
        m = (p1(1)-p0(1))/(p1(0)-p0(0));
        d = p0(1) - m*p0(0);
        xs = (-2*m*d+sqrt(pow(2*m*d,2)-4*(pow(m,2)+1)*(pow(d,2)-pow(r,2))))/(pow(m,2)+1)/2;
        ys = m*xs + d;
    }else{ // left line
        m = (p1(1)-p0(1))/(p1(0)-p0(0));
        d = p0(1) - m*p0(0);
        xs = (-2*m*d-sqrt(pow(2*m*d,2)-4*(pow(m,2)+1)*(pow(d,2)-pow(r,2))))/(pow(m,2)+1)/2;
        ys = m*xs + d;
    }
    ps(0) = xs;
    ps(1) = ys;
    return ps+xc;
}

/********************************************************* 
**********************************************************/
MatrixXd Keyhole::sweep_circle_(Vector2d t1, Vector2d t2, Vector2d xc, double ang_inc, double r){
    t1 = t1 - xc;
    t2 = t2 - xc;
    double ang1 = atan2(t1(1),t1(0));
    double ang2 = atan2(t2(1),t2(0));
    double delta_ang;
    int inc_num=10;

    if((ang1>=0)&&(ang2>=0)){
        if(ang2>=ang1){
            delta_ang = KPI*2-(ang2-ang1);
            ang2 = ang2 - KPI*2;
        }else{
            delta_ang = ang1-ang2;
        }
        inc_num = (int)ceil(delta_ang/ang_inc) + 1;
    }else if((ang1>=0)&&(ang2<0)){
        delta_ang = ang1-ang2;
        inc_num = (int)ceil(delta_ang/ang_inc) + 1;
    }else if((ang1<0)&&(ang2>=0)){
        ang2 = ang2 - 2*KPI;
        delta_ang = KPI*2 - (ang2-ang1);
        inc_num = (int)ceil(delta_ang/ang_inc) + 1;
    }else if((ang1<0)&&(ang2<0)){
        if(ang2>=ang1){
            delta_ang = KPI*2+(ang1-ang2);
            ang1 = ang1 + KPI*2;
        }else{
            delta_ang = -(ang2-ang1);
        }
        inc_num = (int)ceil(delta_ang/ang_inc) + 1;
    }
    if(inc_num<5){
        inc_num = 5;
    }
    VectorXd ang_sweep(inc_num);
    VectorXd x(inc_num), y(inc_num);
    MatrixXd xy(inc_num, 2);
    ang_sweep = VectorXd::LinSpaced(inc_num, ang1, ang2);
    x = ang_sweep.array().cos()*r;
    y = ang_sweep.array().sin()*r;

    x.array() += xc(0);
    y.array() += xc(1);

    xy << x, y;
    return xy;
}

/********************************************************* 
**********************************************************/
void Keyhole::get_line_ceofficients_(Vector2d p0, Vector2d p1, bool left, Vector2d &c, double &d){
    Vector2d pv = p1 - p0;
    double m, s;
    if(left){
        s = 1.0;
    }else{
        s = -1.0;
    }
    if(abs(pv(0))<tol_){ //vertical line
        if(pv(1)>0){ //pointing up
            c << -1*s, 0;
            d = (p1(1)+p0(1))/2.0*(s);
        }
        else{
            c << 1*s, 0;
            d = (p1(1)+p0(1))/2.0*(s);
        }
    }else{
        m = pv(1)/pv(0);
        d = p0(1) - m*p0(0);
        c(0) = -m;
        c(1) = 1;
        d = -d;
        if(pv(0)<0){
            c = -c;
            d = -d;
        }
        c *= s;
        d *= s;
    }
}

/********************************************************* 
**********************************************************/
double Keyhole::relu_line_(Vector2d x, Vector2d coe, double d){
  double val = coe.dot(x) + d;
  if(val<0){
    val = 0;
  }
  return val;
}

/********************************************************* 
**********************************************************/
double Keyhole::rbf_(Vector2d x, Vector2d xc, double beta){
//   double val = (xc-x).norm();
  return exp(-beta*pow((xc-x).norm(),2.0));
}

/********************************************************* 
**********************************************************/
double Keyhole::relu_circle_(Vector2d x, Vector2d xc, double r){
    double val = -((x-xc).dot(x-xc) - pow(r,2.0));
    if(val<0){
        val = 0;
    }
    return val;
}

/********************************************************* 
**********************************************************/ 
std::ostream &operator<<(std::ostream &os, const Keyhole &keyhole) {
    os << "xc = [" << keyhole.xc_.transpose() << "]" << endl
       << "r = " << keyhole.r_ << endl
       << "p0 = [" << keyhole.p0_.transpose() << "]" << endl
       << "p1 = [" << keyhole.p1_.transpose() << "]" << endl
       << "p2 = [" << keyhole.p2_.transpose() << "]" << endl
       << "q1 = [" << keyhole.q1_.transpose() << "]" << endl
       << "q2 = [" << keyhole.q2_.transpose() << "]" << endl
       << "c1 = [" << keyhole.c1_.transpose() << "]" << endl
       << "c2 = [" << keyhole.c2_.transpose() << "]" << endl
       << "c3 = [" << keyhole.c3_.transpose() << "]" << endl
       << "c4 = [" << keyhole.c4_.transpose() << "]" << endl
       << "c5 = [" << keyhole.c5_.transpose() << "]" << endl
       << "d1 = " << keyhole.d1_ << endl
       << "d2 = " << keyhole.d2_ << endl
       << "d3 = " << keyhole.d3_ << endl
       << "d4 = " << keyhole.d4_ << endl
       << "d5 = " << keyhole.d5_ << endl
       << "a = [" << keyhole.a_.transpose() << "]" << endl
       ;
    //    << "||p0-xc||=" << (keyhole.p0_ - keyhole.xc_).norm() << endl
    //    << "||p1-xc||=" << (keyhole.p1_ - keyhole.xc_).norm() << endl
    //    << "||p2-xc||=" << (keyhole.p2_ - keyhole.xc_).norm() << endl;
    //    << "t1=" << keyhole.get_intersection_(keyhole.xc_, keyhole.r_, keyhole.p0_, keyhole.p1_) << endl
    //    << "t2=" << keyhole.get_intersection_(keyhole.xc_, keyhole.r_, keyhole.p0_, keyhole.p2_) << endl;

    return os;
}

/********************************************************* 
**********************************************************/ 
bool Keyhole::is_in_(const double &x, const double &x1, const double & x2){
  // check which is bigger
  double x_low, x_high;
  if(x1>x2){
    x_low = x2;
    x_high = x1;
  }else{
    x_low = x1;
    x_high = x2;
  }

  return (x_low<=x) && (x<=x_high);
}

/********************************************************* 
**********************************************************/ 
MatrixXd Keyhole::get_new_safe_(MatrixXd l_safe, Vector2d c, double d){
  int num = l_safe.rows();
  double val;
  Vector2d tmpx;
  MatrixXd new_safe(num, 2);
  int k = 0;
  
  for(int i=0; i<num; i++){
    tmpx << l_safe(i,0), l_safe(i,1);
    val = c.transpose()*tmpx + d;
    if(val>offset_*abs(c(0))){
      new_safe(k,0) = tmpx(0);
      new_safe(k,1) = tmpx(1);
      k++;
    }
  }

  return new_safe.block(0, 0, k, 2);
}

void Keyhole::extend_parameters_(Vector2d xc, double r, Vector2d q1, Vector2d q2, Vector2d &p1, Vector2d &p2){
    double m1, m2, d1, d2, tmp, x_int, y_int, delta;
    
    m1 = (p1(1)-q1(1))/(p1(0)-q1(0));
    d1 = q1(1)-m1*q1(0);

    m2 = (p2(1)-q2(1))/(p2(0)-q2(0));
    d2 = q2(1)-m2*q2(0);

    x_int = (d2-d1)/(m1-m2);
    y_int = x_int*m1+d1;

    // cout << "m1= " << m1 << endl;
    // cout << "d1= " << d1 << endl;
    // cout << "m2= " << m2 << endl;
    // cout << "d2= " << d2 << endl;
    // cout << "x_int= " << x_int << endl;
    // cout << "y_int= " << y_int << endl;

    if((is_in_(x_int, q1(0), p1(0)))&&(!is_in_(x_int, q2(0), p2(0)))){ // line 2 extension interset lin 1
        delta = x_int - p2(0);
        if(abs(delta)<0.25){
            if(delta>0){
                tmp = x_int + 1.0;
                p2 << tmp, m2*tmp+d2;
            }else{
                tmp = x_int - 1.0;
                p2 << tmp, m2*tmp+d2;   
            }
            // cout << "line 2 extended" << endl;
        }
    }else if((!is_in_(x_int, q1(0), p1(0)))&&(is_in_(x_int, q2(0), p2(0)))){ // line 1 extension interset lin 2
        delta = x_int - p1(0);
        if(abs(delta)<0.25){
            if(delta>0){
                tmp = x_int + 1.0;
                p1 << tmp, m1*tmp+d1;
            }else{
                tmp = x_int - 1.0;
                p1 << tmp, m1*tmp+d1;   
            }
            // cout << "line 1 extended" << endl;
        }
    }
}

void Keyhole::correct_corner_points_(MatrixXd &c_safe, MatrixXd &l1_safe, MatrixXd &l2_safe){

    MatrixXd ctmp, l1tmp, l2tmp;

    ctmp = removeRow_(c_safe, 0);
    ctmp = removeRow_(ctmp, ctmp.rows()-1);

    l1tmp = removeRow_(l1_safe, 0);
    l2tmp = removeRow_(l2_safe, 0);

    c_safe = ctmp;
    l1_safe = l1tmp;
    l2_safe = l2tmp;
}

MatrixXd Keyhole::removeRow_(MatrixXd &mat, const int &rowNum){
  int rowSize = mat.rows();
  int colSize = mat.cols();
  MatrixXd tmp = mat.block(rowNum+1, 0, rowSize-(rowNum+1), colSize);

  MatrixXd tmp2(rowSize-1, colSize);
  tmp2 << mat.block(0, 0, rowNum, colSize), tmp;

  return tmp2;
}

}
}
