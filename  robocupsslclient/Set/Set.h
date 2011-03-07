#ifndef SET_H
#define SET_H

#include <iostream>
#include <math.h>

/**
 *  @author Kamil Muszynski i Maciej Gąbka
 *  @brief klasa pomocnicza reprezentujaca zbior krzywizn pomiędzy cmin a angmax
 */
class Set{
public:
    friend std::ostream & operator<<(std::ostream & o, const Set &s);

    /** @brief konstruktor klasy, tworzy obiekt reprezentujący przedział angmin, angmax
     * @param angmin dolne ograniczenie przedziału
     * @param angmax górne ograniczenie przedziału
     */
    Set(double angmin,double angmax,double d);
    ~Set();
    Set(const Set & );
    //void display();
    Set & operator=(const Set &);
    ///zwraca true jeśli dwa zbiory są rozłączne
    bool areSeparated(Set& set);
    ///zwraca true jesli zbiór wskazywany przez this zawiera się w set
    bool isIncluded(Set& set);
    ///zwraca true jeśli zbiór wskazywany przez this zawiera set
    bool include(Set& set);
    ///zwraca true jesli zbiory mają część wspólną
    bool partlyInclude(Set& set);
    /// zwraca true jesli zbiór jest węższy niż 0.01
    bool isNotCorrect();
    ///
    inline double width(){
        return fabs(angmax-angmin);
    }

    double angmin,angmax;
    double d;
};

std::ostream & operator<<(std::ostream & o, const Set &s);
#endif // SET_H
