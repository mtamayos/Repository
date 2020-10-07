# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 02:05:57 2019

@author: sergi
"""

import numpy as np

def init(L, C, Nz, numIter):
    """
    Inicializa y devuelve los vectores y valores z,t,dz y dt.
    """
    
    # Crear una lista de posiciones de Nz nodos (0,1,...,Nz-1)
    z = np.linspace(0,L,Nz)
    
    dz = z[1] - z[0]               # Longitud del paso en Z
    dt = np.sqrt(C)*dz            # Duracion del paso en T
    
    # Crear una lista de posiciones de Nt nodos (0,1,...,Nt-1)
    t = np.linspace(0, numIter*dt, numIter)
    
    ### Redefinir  dt para que sea compatible con t
    dt = t[1] - t[0]
    
    return (z,t,dz,dt)

def solve(dz, dt, z, numIter, Nz, g, mu, u0, v0, f):
    """
    Soluciona la ecuación diferencial de la cuerda colgando
    """
    C = (dt/dz)**2
    
    ### Vector de soluciones
    U = np.zeros((numIter, Nz))
    
    # Cargar las codiciones iniciales (n=0)
    U[0,:] = u0
    
    # Extremo fijo
    U[0,-1] = 0
    
    # Solucion para n=0 e i=0
    U[1,0]= 0.5*g*C*dz*(U[0,1] - U[0,0]) + U[0,0] + dt*v0[0]\
            -dt**2*f[0,0]/mu
    
    # Solución para n=0
    U[1,1:-1] = 0.5*g*C*((U[0,2:]- U[0,1:-1])*(z[1:-1]+0.5*dz)\
                       - (U[0,1:-1]-U[0,:-2])*(z[1:-1]-0.5*dz))\
                + U[0,1:-1] + dt*v0[1:-1] - 0.5*dt**2*f[0,1:-1]/mu
    
    ### Solucionar para el resto de tiempos
    for n in range(2,numIter):
        
        # Solucion para i=0
        U[n,0] = g*C*dz*(U[n-1,1] - U[n-1,0]) + 2*U[n-1,0]\
                 - U[n-2, 0] - dt**2*f[n-1,0]/mu
        
        # Solucion para el resto de nodos
        U[n,1:-1] = g*C*((U[n-1,2:]- U[n-1,1:-1])*(z[1:-1]+0.5*dz)\
                       - (U[n-1,1:-1]-U[n-1,:-2])*(z[1:-1]-0.5*dz))\
                    + 2*U[n-1,1:-1] - U[n-2,1:-1] - dt**2*f[n-1,1:-1]/mu
    return U