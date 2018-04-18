#pragma once

class Hallway {
public:
  struct Edge {
    enum class Side {
      Left,
      Right
    };

    Hallway* hallway;
    float position;
    Side side;
  };

  Hallway(float width, float length);

  float width() const;
  void width(float width);

  float length() const;
  void length(float length);

  const std::vector<const Edge*>& connections() const;
  void addConnection(const Edge* hallway);

private:
  float m_width, m_length;
  std::vector<const Hallway*> m_connections;
};

class Floorplan {
  

};
