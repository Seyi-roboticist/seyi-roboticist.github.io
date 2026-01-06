---
layout: default
---
<ul>
{% for project in site.projects %}
  <li>{{ project.title }} - {{ project.date }}</li>
{% endfor %}
</ul>
<h2>Sorted</h2>
<ul>
{% for project in site.projects | sort: "date" | reverse %}
  <li>{{ project.title }} - {{ project.date }}</li>
{% endfor %}
</ul>
