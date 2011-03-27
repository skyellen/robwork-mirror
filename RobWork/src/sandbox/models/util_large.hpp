
// Euler-Bernoulli differential equation
struct RHSEulerBernoulli {
        double _F, _M, _EI, _L;
        RHSEulerBernoulli(double F, double M, double EI, double L) : _F(F), _M(M), _EI(EI), _L(L) {}
        void operator()(const double x, const VecDoub& y, VecDoub& dydx) {
            const double &k = y[0], &a = y[1];
            dydx[0] = (_M - _F*std::cos(a)) / _EI;
            dydx[1] = k;
            dydx[2] = std::cos(a);
            dydx[3] = std::sin(a);
        }
};

// Backward Euler-Bernoulli differential equation
struct RHSEulerBernoulliBack {
        double _F, _M, _EI, _L;
        RHSEulerBernoulliBack(double F, double M, double EI, double L) : _F(F), _M(M), _EI(EI), _L(L) {}
        void operator()(const double x, const VecDoub& y, VecDoub& dydx) {
            const double &k = y[0], &a = y[1];
            dydx[0] = -(_M - _F*std::cos(a)) / _EI;
            dydx[1] = -k;
            dydx[2] = -std::cos(a);
            dydx[3] = -std::sin(a);
        }
};


// One-dimensional function evaluation
class Shoot {
    public:        
        Shoot(double x1, double x2, unsigned int n,
                double F, double M, double EI, double L) :  _out(n), _rhs(F, M, EI, L),
                _x1(x1), _x2(x2), _atol(0.001), _rtol(0.001),
                _h1((_x2-_x1)/(double)n), _hmin(0.0) {}
        double operator()(double kappa_init) {
            // Initial conditions
            VecDoub ystart(4, 0.0);
            ystart[0] = kappa_init;
            // Integrate
            Odeint<StepperDopr853<RHSEulerBernoulli> > ode(ystart, _x1, _x2, _atol, _rtol, _h1, _hmin, _out, _rhs);
            ode.integrate();
            // Get tip configuration for kappa
            const double& kappa_end = _out.ysave[0][_out.count-1];
            
            // Return score
            return _rhs._M/_rhs._EI - kappa_end;
        }
        
        Output _out;
        
    private:
        RHSEulerBernoulli _rhs;
        double _x1, _x2, _atol, _rtol, _h1, _hmin;
};


class ShootBack {
    public:        
        ShootBack(double x1, double x2, unsigned int n,
                  double a, double y,
                  double EI, double L) :  _out(n), _a(a), _y(y), _EI(EI), _L(L),
                                          _x1(x1), _x2(x2), _atol(0.001), _rtol(0.001),
                                          _h1((_x2-_x1)/(double)n), _hmin(0.0) {}
        
        double operator()(const VecDoub& x) {
            const double &Finit = x[0], &Minit = x[1], &zinit = x[2];
            
            RHSEulerBernoulliBack rhs(Finit, Minit, _EI, _L);
            
            // Initial conditions at the tip
            VecDoub ystart(4);
            ystart[0] = Minit / _EI;
            ystart[1] = _a;
            ystart[2] = zinit;
            ystart[3] = _y;
            // Integrate backwards to the root
            Odeint<StepperDopr853<RHSEulerBernoulliBack> > ode(ystart, _x1, _x2, _atol, _rtol, _h1, _hmin, _out, rhs);
            ode.integrate();
            // Get root configuration
            const double& a_root = _out.ysave[1][_out.count-1];
            const double& z_root = _out.ysave[2][_out.count-1];
            const double& y_root = _out.ysave[3][_out.count-1];
            
            // Return score
            return std::abs(a_root) + std::abs(z_root) + std::abs(y_root);
        }
        
        Output _out;
        
    private:        
        double _a, _y, _EI, _L;
        double _x1, _x2, _atol, _rtol, _h1, _hmin;
};

