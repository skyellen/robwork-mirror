/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "ClarkHullND.hpp"

/*
 * Ken Clarkson wrote this.  Copyright (c) 1995 by AT&T..
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY.  IN PARTICULAR, NEITHER THE AUTHORS NOR AT&T MAKE ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <assert.h>
#include <string.h>


#define push(x) *(st+tms++) = x;
#define pop(x)  x = *(st + --tms);

#define MAXDIM 10

namespace {

typedef void* visit_func(simplex *, void *);
typedef int test_func(simplex *, int, void *);
typedef void out_func(point *, int, FILE*, int);

    typedef double Coord;
    typedef Coord* point;
    int pdim;

    #define MAXBLOCKS 10000

    typedef point site;
    typedef Coord* normalp;
    point   site_blocks[MAXBLOCKS];
    int num_blocks;


    struct basis_s {
        struct basis_s *next; /* free list */
        int ref_count;  /* storage management */
        int lscale;    /* the log base 2 of total scaling of vector */
        Coord sqa, sqb; /* sums of squared norms of a part and b part */
        Coord vecs[1]; /* the actual vectors, extended by malloc'ing bigger */
    };

    struct simplex;
    struct neighbor {
        site vert; /* vertex of simplex */
        simplex *simp; /* neighbor sharing all vertices but vert */
        basis_s *basis; /* derived vectors */
    };

    struct simplex {
        struct simplex *next;   /* free list */
        long visit;     /* number of last site visiting this simplex */
    /*  float Sb; */
        short mark;
        basis_s* normal;    /* normal vector pointing inward */
        neighbor peak;      /* if null, remaining vertices give facet */
        neighbor neigh[1];  /* neighbors of simplex */
    };

    typedef struct fg_node fg;
    typedef struct tree_node Tree;
    struct tree_node {
        Tree *left, *right;
        site key;
        int size;   /* maintained to be the number of nodes rooted here */
        fg *fgs;
        Tree *next; /* freelist */
    };

    struct fg_node {
        Tree *facets;
        double dist, vol;   /* of Voronoi face dual to this */
        fg *next;       /* freelist */
        short mark;
        int ref_count;
    };

    //typedef void* visit_func(simplex *, void *);
    typedef int test_func(simplex *, int, void *);
    typedef void out_func(point *, int, FILE*, int);

    point get_another_site(void);

    void* visit_triang_gen(simplex *, visit_func, test_func*);
    void* visit_triang(simplex *, visit_func);
    void* visit_hull(simplex *, visit_func);

    neighbor *op_simp(simplex *a, simplex *b);

    neighbor *op_vert(simplex *a, site b);

    simplex *new_simp(void);

    void buildhull(simplex *);

    site p;
    long pnum;

    int rdim,   /* region dimension: (max) number of sites specifying region */
        cdim,   /* number of sites currently specifying region */
        site_size, /* size of malloc needed for a site */
        point_size;  /* size of malloc needed for a point */

    long vnum;
    long ss;
    simplex **st;



    ////// NOW FOR THE IMPLEMENTATION


    void* visit_triang_gen(simplex *s, visit_func *visit, test_func *test) {
        /*
         * starting at s, visit simplices t such that test(s,i,0) is true,
         * and t is the i'th neighbor of s;
         * apply visit function to all visited simplices;
         * when visit returns nonNULL, exit and return its value
         */
        neighbor *sn;
        void *v;
        simplex *t;
        int i;
        long tms = 0;
        vnum--;
        if (!st)
            st=(simplex**)malloc((ss+MAXDIM+1)*sizeof(simplex*));
        if (s)
            push(s);
        while (tms) {

            if (tms>ss) {
                //DEBEXP(-1,tms);
                st=(simplex**)realloc(st,
                    ((ss+=ss)+MAXDIM+1)*sizeof(simplex*));
            }
            pop(t);
            if (!t || t->visit == vnum) continue;
            t->visit = vnum;
            if (v=(*visit)(t,0)) {
                return v;
            }
            for (i=-1,sn = t->neigh-1;i<cdim;i++,sn++)
                if ((sn->simp->visit != vnum) && sn->simp && test(t,i,0))
                    push(sn->simp);
        }
        return NULL;
    }

    namespace {
        int truet(simplex *s, int i, void *dum) {
            return 1;
        }

        int hullt(simplex *s, int i, void *dummy) {
            return i>-1;
        }

        void* facet_test(simplex *s, void *dummy) {
            return (!s->peak.vert) ? s : NULL;
        }
    }



    void* visit_triang(simplex *root, visit_func *visit){
        /* visit the whole triangulation */
        return visit_triang_gen(root, visit, truet);
    }

    void* visit_hull(simplex *root, visit_func *visit)
    /* visit all simplices with facets of the current hull */
    {
        return visit_triang_gen(visit_triang(root, &facet_test),visit, hullt);
    }

    #define lookup(a,b,what,whatt)                      \
    {                                   \
        int i;                              \
        neighbor *x;                            \
        for (i=0, x = a->neigh; (x->what != b) && (i<cdim) ; i++, x++); \
        if (i<cdim)                         \
            return x;                       \
        else {                              \
            fprintf(DFILE,"adjacency failure,op_" #what ":\n"); \
            DEBTR(-10)                      \
            print_simplex_f(a, DFILE, &print_neighbor_full);    \
            print_##whatt(b, DFILE);                \
            fprintf(DFILE,"---------------------\n");       \
            print_triang(a,DFILE, &print_neighbor_full);        \
            exit(1);                        \
            return 0;                       \
        }                               \
    }                                   \


    neighbor *op_simp(simplex *a, simplex *b) {lookup(a,b,simp,simplex)}
        /* the neighbor entry of a containing b */

    neighbor *op_vert(simplex *a, site b) {lookup(a,b,vert,site)}
        /* the neighbor entry of a containing b */


    void connect(simplex *s) {
    /* make neighbor connections between newly created simplices incident to p */

        site xf,xb,xfi;
        simplex *sb, *sf, *seen;
        int i;
        neighbor *sn;

        if (!s) return;
        assert(!s->peak.vert
            && s->peak.simp->peak.vert==p
            && !op_vert(s,p)->simp->peak.vert);
        if (s->visit==pnum) return;
        s->visit = pnum;
        seen = s->peak.simp;
        xfi = op_simp(seen,s)->vert;
        for (i=0, sn = s->neigh; i<cdim; i++,sn++) {
            xb = sn->vert;
            if (p == xb) continue;
            sb = seen;
            sf = sn->simp;
            xf = xfi;
            if (!sf->peak.vert) {   /* are we done already? */
                sf = op_vert(seen,xb)->simp;
                if (sf->peak.vert) continue;
            } else do {
                xb = xf;
                xf = op_simp(sf,sb)->vert;
                sb = sf;
                sf = op_vert(sb,xb)->simp;
            } while (sf->peak.vert);

            sn->simp = sf;
            op_vert(sf,xf)->simp = s;

            connect(sf);
        }

    }



    static simplex *make_facets(simplex *seen) {
    /*
     * visit simplices s with sees(p,s), and make a facet for every neighbor
     * of s not seen by p
     */

        simplex *n;
        static simplex *ns;
        neighbor *bn;
        int i;


        if (!seen) return NULL;
        DEBS(-1) assert(sees(p,seen) && !seen->peak.vert); EDEBS
        seen->peak.vert = p;

        for (i=0,bn = seen->neigh; i<cdim; i++,bn++) {
            n = bn->simp;
            if (pnum != n->visit) {
                n->visit = pnum;
                if (sees(p,n)) make_facets(n);
            }
            if (n->peak.vert) continue;
            copy_simp(ns,seen);
            ns->visit = 0;
            ns->peak.vert = 0;
            ns->normal = 0;
            ns->peak.simp = seen;
    /*      ns->Sb -= ns->neigh[i].basis->sqb; */
            NULLIFY(basis_s,ns->neigh[i].basis);
            ns->neigh[i].vert = p;
            bn->simp = op_simp(n,seen)->simp = ns;
        }
        return ns;
    }



    static simplex *extend_simplices(simplex *s) {
    /*
     * p lies outside flat containing previous sites;
     * make p a vertex of every current simplex, and create some new simplices
     */

        int i,
            ocdim=cdim-1;
        simplex *ns;
        neighbor *nsn;

        if (s->visit == pnum) return s->peak.vert ? s->neigh[ocdim].simp : s;
        s->visit = pnum;
        s->neigh[ocdim].vert = p;
        NULLIFY(basis_s,s->normal);
        NULLIFY(basis_s,s->neigh[0].basis);
        if (!s->peak.vert) {
            s->neigh[ocdim].simp = extend_simplices(s->peak.simp);
            return s;
        } else {
            copy_simp(ns,s);
            s->neigh[ocdim].simp = ns;
            ns->peak.vert = NULL;
            ns->peak.simp = s;
            ns->neigh[ocdim] = s->peak;
            inc_ref(basis_s,s->peak.basis);
            for (i=0,nsn=ns->neigh;i<cdim;i++,nsn++)
                nsn->simp = extend_simplices(nsn->simp);
        }
        return ns;
    }


    static simplex *search(simplex *root) {
    /* return a simplex s that corresponds to a facet of the
     * current hull, and sees(p, s) */

        simplex *s;
        static simplex **st;
        static long ss = MAXDIM;
        neighbor *sn;
        int i;
        long tms = 0;

        if (!st) st = (simplex **)malloc((ss+MAXDIM+1)*sizeof(simplex*));
        push(root->peak.simp);
        root->visit = pnum;
        if (!sees(p,root))
            for (i=0,sn=root->neigh;i<cdim;i++,sn++) push(sn->simp);
        while (tms) {
            if (tms>ss)
                st=(simplex**)realloc(st,
                    ((ss+=ss)+MAXDIM+1)*sizeof(simplex*));
            pop(s);
            if (s->visit == pnum) continue;
            s->visit = pnum;
            if (!sees(p,s)) continue;
            if (!s->peak.vert) return s;
            for (i=0, sn=s->neigh; i<cdim; i++,sn++) push(sn->simp);
        }
        return NULL;
    }



    void buildhull (simplex *root) {

        while (cdim < rdim) {
            p = get_another_site();
            if (!p) return;
            if (out_of_flat(root,p))
                extend_simplices(root);
            else
                connect(make_facets(search(root)));
        }
        while (p = get_another_site())
            connect(make_facets(search(root)));
    }

    point get_another_site(void) {

        static int scount =0;
        point pnext;

        if (!(++scount%1000)) {fprintf(DFILE,"site %d...", scount);}
    /*  check_triang(); */
        pnext = (*get_site)();
        if (!pnext) return NULL;
        pnum = site_num(pnext)+2;
        return pnext;
    }

}


