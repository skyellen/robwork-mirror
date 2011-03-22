/*******************************************************************************
 * Copyright (c) 2011 IBM Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     IBM Corporation - initial API and implementation
 *******************************************************************************/
struct StepperBase {
	Doub &x;
	Doub xold;
	VecDoub &y,&dydx;
	Doub atol,rtol;
	bool dense;
	Doub hdid;
	Doub hnext;
	Doub EPS;
	Int n,neqn;
	VecDoub yout,yerr;
	StepperBase(VecDoub_IO &yy, VecDoub_IO &dydxx, Doub &xx, const Doub atoll,
		const Doub rtoll, bool dens) : x(xx),y(yy),dydx(dydxx),atol(atoll),
		rtol(rtoll),dense(dens),n(y.size()),neqn(n),yout(n),yerr(n) {}
};
