# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality:
        * Method 1: Create a member function called normalize that will take no arguments and modifies the Vector2D object it is called on. This method will use the magnitude of the Vector2D to divide each component of the Vector2D by it.
        * Method 2: Create a free function called normalize that takes a Vector2D object as an argument and returns a new Vector2D object that is the normalized version of the input Vector2D.
        * Method 3: Create a member function called normalize that will take no arguments and returns a new Vector2D object that is the unit vector of the Vector2D object it is called on.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
        * Method 1 has the advantage of being simple to use, since it modifies the object directly.
        However, it can lead to unexpected behavior if the user is not aware that the original object is being modified.
        * Method 2 is more self-explanatory and guarantees that the original object is not modified.
        * Method 3 is less likely to be confused with the original object, but it would require the user to save the output in a new variable.

   - Which of the methods would you implement and why?
        * Method 2 is the one I would implement because it is more self-explanatory and guarantees that the original object is not modified.

2. What is the difference between a class and a struct in C++?
    * In C++, a struct is a class with default public members and base classes, whereas a class has public and private members. In general, structs are used for grouping data together and classes are used for grouping data and functions together.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
    * Vector2D is a struct because it only contains data members (x and y), and no member functions. According to C++ Core Guidelines, structs are preferred over classes when the struct is only used to group data together. Transform2D is a class because it contains both data members (tran and rot) and member functions (e.g. translation, rotation).

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
    * It prevents implicit type conversions. According to C++ Core Guidelines, single-argument constructors should be declared explicit to avoid potential bugs.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
        * Transform2D::inv() is declared const because it only returns a new Transform2D object and it does not modify the original object. According to the C++ Core Guidelines, member functions that don't modify the object should be declared const. On the other hand, Transform2D::operator*=() modifies the original object, hence it is not declared const.
