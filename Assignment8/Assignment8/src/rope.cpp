#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, 
        //ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
// //        }

        Vector2D unitDist = (end - start)/(num_nodes-1);
        Mass *startM = new Mass(start,node_mass,false);
        masses.push_back(startM);
        for(int i =1 ; i < num_nodes; i++){
            auto preM = masses[i-1];
            Vector2D curPos = preM->position + unitDist;
            Mass *curMass = new Mass(curPos,node_mass,false);
            masses.push_back(curMass);
            Spring *curSpring = new Spring(preM,curMass,k);
            springs.push_back(curSpring);
            std::cout<<"i "<<i<<std::endl;
        }

        for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to 
            //calculate the force on a node
            Mass *a = s->m1;
            Mass *b = s->m2;
            double dist = (b->position-a->position).norm();
            Vector2D fba = (s->k)*((b->position-a->position)/dist)
                            *(dist-s->rest_length);
            Vector2D fab = (s->k)*((a->position-b->position)/dist)
                            *(dist-s->rest_length);
            a->forces += fba;
            b->forces += fab;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, 
                // TODO (Part 2): Add global damping
                // then compute the new velocity and position
                m->forces += m->mass * gravity;
                float k_d_global = 0.01;
                m->forces += - k_d_global * m->velocity;

                Vector2D accelerate = m->forces/m->mass;
                m->velocity += accelerate*delta_t;
                m->position += m->velocity*delta_t;

            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ab = s->m2->position - s->m1->position;
            Vector2D f = s->k *  (ab / ab.norm()) * (ab.norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces -= f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;

                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D lastposition = m->position;
                // TODO (Part 4): Add global Verlet damping
                float dampfactor = 0.00005;
                m->position = m->position +  (1 - dampfactor) * (m->position - m->last_position) + a * delta_t *delta_t;
                m->last_position = lastposition;
            }
            m->forces =  Vector2D(0,0);
        }
    }
}
