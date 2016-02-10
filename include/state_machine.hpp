#ifndef BASIC_STATE_MACHINE_HPP
#define BASIC_STATE_MACHINE_HPP

#include <map>
#include <vector>

// namespace dual_manipulation{
//     namespace state_manager{
        
  
        template <class state_type, class transition_type>
        class state_machine
        {
            public:
                std::map<state_type, std::map<transition_type, state_type>> Grafo;

                state_type evolve_state_machine(state_type current_state, transition_type command)
                {
                    if (Grafo.count(current_state)) /**if exist this state*/
                    {
                        if (Grafo.at(current_state).count(command)) /**if exist this transition in this state*/
                            return Grafo.at(current_state).at(command); /**if exist return the next state*/
                    }
                    return current_state;
                }
                
                void insert(std::vector< std::tuple< state_type, transition_type, state_type > > table)
                {
                    for (auto row:table)
                        insert(row);
                }
                
                void insert(std::tuple< state_type, transition_type, state_type > row)
                {
                    Grafo[std::get<0>(row)][std::get<1>(row)]=std::get<2>(row);
                }
                
                state_machine(){}
        };
        
    // }
// }
#endif // BASIC_STATE_MACHINE_HPP